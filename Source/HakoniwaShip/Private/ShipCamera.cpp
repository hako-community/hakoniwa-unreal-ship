// ShipCamera.cpp

#include "ShipCamera.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Dom/JsonObject.h"
#include "Engine/TextureRenderTarget2D.h"
#include "GameFramework/Actor.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "PduManager.h"
#include "RenderingThread.h"
#include "RHIGPUReadback.h"
#include "ShipFrame.h"
#include "TextureResource.h"

#include "pdu_convertor.hpp"
#include "sensor_msgs/pdu_cpptype_conv_Image.hpp"

DEFINE_LOG_CATEGORY_STATIC(LogShipCamera, Log, All);

/** 非同期の読み出し（ShipOcean の海面の検証と同じ方式）。描画スレッドが GPU の写しを Pixels に移して bReady を立てる */
struct FShipCameraReadback
{
	TUniquePtr<FRHIGPUTextureReadback> Readback;
	TArray<FColor> Pixels;   // BGRA（PF_B8G8R8A8）
	std::atomic<bool> bReady { false };
	double Stamp = 0.0;
	int32 Frames = 0;
};

namespace
{
	FVector JsonVec3(const TSharedPtr<FJsonObject>& O, const TCHAR* Key)
	{
		const TArray<TSharedPtr<FJsonValue>>* A = nullptr;
		if (!O->TryGetArrayField(Key, A) || A->Num() != 3)
		{
			return FVector::ZeroVector;
		}
		return FVector((*A)[0]->AsNumber(), (*A)[1]->AsNumber(), (*A)[2]->AsNumber());
	}
	constexpr int32 MaxPending = 4;
}

bool UShipCamera::Init(AActor* Ship, const TSharedPtr<FJsonObject>& Spec, double PhaseSec)
{
	Robot = Spec->GetStringField(TEXT("robot"));
	PduName = Spec->GetStringField(TEXT("pdu"));
	FrameId = Spec->GetStringField(TEXT("frame_id"));
	Width = Spec->GetIntegerField(TEXT("width"));
	Height = Spec->GetIntegerField(TEXT("height"));
	RateHz = Spec->GetNumberField(TEXT("rate_hz"));
	Phase = PhaseSec;
	if (Spec->GetStringField(TEXT("encoding")) != TEXT("rgb8") || Width <= 0 || Height <= 0 || RateHz <= 0.0)
	{
		UE_LOG(LogShipCamera, Error, TEXT("%s: unsupported camera spec"), *FrameId);
		return false;
	}
	Target = NewObject<UTextureRenderTarget2D>(this);
	// ポストプロセス後の色を sRGB で 8 bit に（GPU が書くときに符号化する）。RTF_RGBA8 だと線形のまま入り、画素の値が 2 段ほど暗かった
	Target->RenderTargetFormat = RTF_RGBA8_SRGB;
	Target->ClearColor = FLinearColor::Black;
	Target->InitAutoFormat(Width, Height);
	Target->UpdateResourceImmediate(true);

	Capture = NewObject<USceneCaptureComponent2D>(Ship, *(TEXT("Camera_") + FrameId));
	Capture->SetupAttachment(Ship->GetRootComponent());
	Capture->SetRelativeLocation(ShipFrame::LocalToUnreal(JsonVec3(Spec, TEXT("origin"))));
	Capture->SetRelativeRotation(FRotator::ZeroRotator);   // 船首方向・水平（Unreal の +X = 船首）
	Capture->FOVAngle = Spec->GetNumberField(TEXT("fov_deg"));
	// 取り付け位置はカメラの筐体（上部構造物のメッシュの一部）の中なので、手前を切って筐体を写さない
	double NearClip = 0.0;
	if (Spec->TryGetNumberField(TEXT("near_clip_m"), NearClip) && NearClip > 0.0)
	{
		Capture->bOverride_CustomNearClippingPlane = true;
		Capture->CustomNearClippingPlane = NearClip * 100.0;
	}
	Capture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	Capture->bCaptureEveryFrame = false;
	Capture->bCaptureOnMovement = false;
	Capture->bAlwaysPersistRenderingState = true;   // 時間方向の処理（にじみ・ちらつき止め）の履歴を保つ
	// 露出は画面と同じ（場面の無限の範囲のポストプロセスの EV100 固定。最小 = 最大なので適応の速さによらない）
	Capture->ShowFlags.SetEyeAdaptation(true);
	Capture->TextureTarget = Target;
	Capture->RegisterComponent();
	Ship->AddInstanceComponent(Capture);

	FParse::Value(FCommandLine::Get(), TEXT("-ShipCameraDump="), DumpDir);
	UE_LOG(LogShipCamera, Log, TEXT("camera %s -> %s/%s: %dx%d rgb8, %.1f Hz, fov %.0f deg, at %s m (FLU)"),
		*FrameId, *Robot, *PduName, Width, Height, RateHz, Capture->FOVAngle, *JsonVec3(Spec, TEXT("origin")).ToString());
	return true;
}

void UShipCamera::MaybeCapture(double T)
{
	const double Period = 1.0 / RateHz;
	if (NextCapture < 0.0)
	{
		// 最初の区切り：k·Period + Phase のうち T 以上で最小のもの
		NextCapture = FMath::CeilToDouble((T - Phase) / Period) * Period + Phase;
	}
	if (T < NextCapture || !Capture)
	{
		return;
	}
	while (NextCapture <= T)
	{
		NextCapture += Period;
	}
	if (Pending.Num() >= MaxPending)
	{
		++Dropped;   // GPU の受け取りが追いつかない。撮らずに次の区切りへ
		return;
	}
	Capture->CaptureScene();
	TSharedPtr<FShipCameraReadback, ESPMode::ThreadSafe> R = MakeShared<FShipCameraReadback, ESPMode::ThreadSafe>();
	R->Readback = MakeUnique<FRHIGPUTextureReadback>(TEXT("ShipCamera"));
	R->Stamp = T;
	FTextureRenderTargetResource* Res = Target->GameThread_GetRenderTargetResource();
	ENQUEUE_RENDER_COMMAND(ShipCameraCopy)([R, Res](FRHICommandListImmediate& RHICmdList)
	{
		R->Readback->EnqueueCopy(RHICmdList, Res->GetRenderTargetTexture());
	});
	Pending.Add(R);
	++Captured;
}

void UShipCamera::Poll(UPduManager* Pdu)
{
	// 撮った順に受け取る（先のものが届くまで後のものは書かない。時刻の順を守る）
	while (Pending.Num() > 0)
	{
		TSharedPtr<FShipCameraReadback, ESPMode::ThreadSafe> R = Pending[0];
		if (!R->bReady.load())
		{
			++R->Frames;
			const int32 W = Width, H = Height;
			ENQUEUE_RENDER_COMMAND(ShipCameraPoll)([R, W, H](FRHICommandListImmediate&)
			{
				if (R->bReady.load() || !R->Readback->IsReady())
				{
					return;
				}
				int32 Pitch = 0;
				const FColor* Src = static_cast<const FColor*>(R->Readback->Lock(Pitch));
				if (Src)
				{
					R->Pixels.SetNumUninitialized(W * H);
					for (int32 y = 0; y < H; ++y)
					{
						FMemory::Memcpy(&R->Pixels[y * W], Src + y * Pitch, W * sizeof(FColor));
					}
				}
				R->Readback->Unlock();
				R->bReady.store(true);
			});
			return;
		}
		Pending.RemoveAt(0);
		MaxLatencyFrames = FMath::Max(MaxLatencyFrames, R->Frames);
		if (R->Pixels.Num() == Width * Height)
		{
			Write(Pdu, R->Pixels, R->Stamp);
		}
		else
		{
			++WriteFailures;
		}
	}
}

void UShipCamera::Write(UPduManager* Pdu, const TArray<FColor>& Bgra, double Stamp)
{
	HakoCpp_Image Img {};
	const double Sec = FMath::FloorToDouble(Stamp);
	Img.header.stamp.sec = static_cast<int32>(Sec);
	Img.header.stamp.nanosec = static_cast<uint32>(FMath::RoundToDouble((Stamp - Sec) * 1.0e9));
	Img.header.frame_id = TCHAR_TO_UTF8(*FrameId);
	Img.height = Height;
	Img.width = Width;
	Img.encoding = "rgb8";
	Img.is_bigendian = 0;
	Img.step = Width * 3;
	Img.data.resize(static_cast<size_t>(Width) * Height * 3);
	uint64 Sum = 0;
	for (int32 i = 0; i < Width * Height; ++i)
	{
		const FColor& C = Bgra[i];   // FColor のメモリの並びは B, G, R, A
		Img.data[3 * i] = C.R;
		Img.data[3 * i + 1] = C.G;
		Img.data[3 * i + 2] = C.B;
		Sum += (299u * C.R + 587u * C.G + 114u * C.B) / 1000u;
	}
	MeanLuma = static_cast<double>(Sum) / (Width * Height);
	LastStamp = Stamp;
	if (Pdu)
	{
		const int32 Size = Pdu->GetPduSize(Robot, PduName);
		Buffer.SetNumZeroed(FMath::Max(Size, 0));
		hako::pdu::PduConvertor<HakoCpp_Image, hako::pdu::msgs::sensor_msgs::Image> Conv;
		const int N = Size > 0 ? Conv.cpp2pdu(Img, reinterpret_cast<char*>(Buffer.GetData()), Size) : -1;
		if (N <= 0 || !Pdu->FlushPduRawData(Robot, PduName, Buffer))
		{
			if (++WriteFailures == 1)
			{
				UE_LOG(LogShipCamera, Error, TEXT("%s: cannot write %s/%s (pdu size %d, converted %d)"), *FrameId, *Robot, *PduName, Size, N);
			}
			return;
		}
	}
	++Written;
	if (!DumpDir.IsEmpty() && Stamp >= NextDump)
	{
		NextDump = Stamp + 2.0;
		IImageWrapperModule& Wrappers = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		TSharedPtr<IImageWrapper> Png = Wrappers.CreateImageWrapper(EImageFormat::PNG);
		if (Png.IsValid() && Png->SetRaw(Bgra.GetData(), Bgra.Num() * sizeof(FColor), Width, Height, ERGBFormat::BGRA, 8))
		{
			const FString File = FPaths::Combine(DumpDir, FString::Printf(TEXT("%s_%08.3f.png"), *FrameId, Stamp));
			FFileHelper::SaveArrayToFile(Png->GetCompressed(), *File);
		}
	}
}
