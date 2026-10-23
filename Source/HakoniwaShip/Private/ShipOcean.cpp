// ShipOcean.cpp

#include "ShipOcean.h"

#include "Camera/PlayerCameraManager.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "UObject/ConstructorHelpers.h"
#include "ProceduralMeshComponent.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "ShipFrame.h"
#include "ShipOceanShader.h"

DEFINE_LOG_CATEGORY_STATIC(LogShipOcean, Log, All);

namespace
{
	const TCHAR* DefaultNearMaterial = TEXT("/Game/Ship/Generated/M_ShipOcean.M_ShipOcean");
	const TCHAR* DefaultFarMaterial = TEXT("/Game/Ship/Generated/M_ShipOceanFar.M_ShipOceanFar");
	constexpr int32 CheckPixels = 64;       // 真上からの検証の画素数（一辺）。4 m 格子で 256 m 四方
	constexpr double CheckHeightM = 200.0;  // 検証のカメラの高さ [m]
	constexpr double SurfaceTolM = 5.0e-3;  // Godot の wave_shader_parity と同じ許容（単精度の GPU）

	double Wrap2Pi(double A)
	{
		A = FMath::Fmod(A, 2.0 * UE_DOUBLE_PI);
		return A < 0.0 ? A + 2.0 * UE_DOUBLE_PI : A;
	}
}

AShipOcean::AShipOcean()
{
	PrimaryActorTick.bCanEverTick = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Ocean"));
	Near = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Near"));
	Near->SetupAttachment(RootComponent);
	Far = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Far"));
	Far->SetupAttachment(RootComponent);
	for (UPrimitiveComponent* C : { static_cast<UPrimitiveComponent*>(Near), static_cast<UPrimitiveComponent*>(Far) })
	{
		C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		C->SetCastShadow(false);
		C->bAffectDistanceFieldLighting = false;
	}
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Plane(TEXT("/Engine/BasicShapes/Plane.Plane"));
	if (Plane.Succeeded())
	{
		Far->SetStaticMesh(Plane.Object);
	}
}

void AShipOcean::BeginPlay()
{
	Super::BeginPlay();
	const TCHAR* Cmd = FCommandLine::Get();
	FString ParityPath;
	if (FParse::Value(Cmd, TEXT("-ShipWaveParity="), ParityPath))
	{
		RunParity(ParityPath);
	}
	bAutoSurfaceCheck = FParse::Param(Cmd, TEXT("ShipSurfaceCheck"));
	if (!NearMaterial) { NearMaterial = LoadObject<UMaterialInterface>(nullptr, DefaultNearMaterial); }
	if (!FarMaterial) { FarMaterial = LoadObject<UMaterialInterface>(nullptr, DefaultFarMaterial); }
	if (!NearMaterial || !FarMaterial)
	{
		UE_LOG(LogShipOcean, Error, TEXT("ocean materials not found (run scripts\\windows\\ship_scene.ps1 -Build)"));
	}
	CreateWaveTexture();
	BuildMeshes();
	if (!bHasParams)
	{
		// AShipFleet が SetWaveParams するまで（箱庭につないだときは Disturbance が届くまで）は静水
		ShipWave::FParams Calm;
		Calm.hs_m = 0.0;
		SetWaveParams(Calm);
	}
}

void AShipOcean::EndPlay(const EEndPlayReason::Type Reason)
{
	if (SurfaceChecks > 0)
	{
		UE_LOG(LogShipOcean, Log, TEXT("SHIP_SEA surface summary: checks %d points %lld misses %lld max err %.2e m (max |eta| %.2f m)"),
			SurfaceChecks, SurfacePoints, SurfaceMisses, SurfaceMaxErr, SurfaceMaxEta);
	}
	Super::EndPlay(Reason);
}

void AShipOcean::Configure(int32 InNumComponents, double InActiveHs)
{
	NumComponents = FMath::Clamp(InNumComponents, 1, ShipOceanShader::MaxWaves);
	ActiveHs = InActiveHs;
}

void AShipOcean::SetWaveParams(const ShipWave::FParams& P)
{
	ShipWave::FParams Merged = P;
	Merged.num_components = NumComponents;
	if (bHasParams && ShipWave::SameParams(Merged, Params))
	{
		return;
	}
	Params = Merged;
	bHasParams = true;
	bActive = Params.hs_m > ActiveHs;
	Engine.initialize(Params);

	// 見た目だけの細かい波：波長 0.8〜8 m、主波向 ±72°、傾き（k·a）0.05。決まった種で作る（毎回同じ見た目）
	Ripples.Reset();
	FRandomStream Rng(20261023);
	const int32 NR = FMath::Clamp(RippleCount, 0, ShipOceanShader::MaxRipples);
	for (int32 j = 0; j < NR; ++j)
	{
		const double L = 0.8 * FMath::Pow(10.0, static_cast<double>(j) / FMath::Max(1, NR - 1));
		const double K = 2.0 * UE_DOUBLE_PI / L;
		const double Dir = Params.wave_dir_rad + (Rng.FRand() - 0.5) * 0.8 * UE_DOUBLE_PI;
		FRipple R;
		R.Kx = K * FMath::Cos(Dir);
		R.Ky = K * FMath::Sin(Dir);
		R.Amp = 0.05 / K;
		R.Omega = FMath::Sqrt(ship::environment::GRAVITY * K);
		R.Phase = Rng.FRand() * 2.0 * UE_DOUBLE_PI;
		Ripples.Add(R);
	}
	UE_LOG(LogShipOcean, Log, TEXT("wave: active=%d Hs %.2f m Tp %.2f s dir %.3f rad gamma %.2f seed %u depth %.1f m tide %.2f m, %d components, %d ripples"),
		bActive, Params.hs_m, Params.tp_s, Params.wave_dir_rad, Params.gamma, Params.seed, Params.water_depth_m, Params.tide_level_m,
		ComponentCount(), Ripples.Num());
}

void AShipOcean::SetPhysicsTime(double T)
{
	TimeSec = T;
	bPhysicsTime = true;
}

double AShipOcean::ElevationAt(double East, double North, double T) const
{
	return bActive ? Engine.get_elevation(East, North, T) : Params.tide_level_m;
}

void AShipOcean::CreateWaveTexture()
{
	WaveTex = UTexture2D::CreateTransient(ShipOceanShader::TexWidth, 2, PF_A32B32G32R32F, TEXT("ShipWaveTable"));
	WaveTex->Filter = TF_Nearest;
	WaveTex->SRGB = false;
	WaveTex->AddressX = TA_Clamp;
	WaveTex->AddressY = TA_Clamp;
	WaveTex->NeverStream = true;
	WaveTex->UpdateResource();
}

void AShipOcean::BuildMeshes()
{
	// 波面メッシュ：中心が原点の (Cells+1)² 頂点の格子。三角形は上向き（UKismetProceduralMeshLibrary::CalculateTangentsForMesh の向き）
	StepM = MeshSizeM / Cells;
	const int32 W = Cells + 1;
	const double Half = MeshSizeM * 0.5;
	TArray<FVector> V;
	TArray<FVector> Nrm;
	TArray<int32> Tri;
	V.Reserve(W * W + 2);
	Nrm.Reserve(W * W + 2);
	Tri.Reserve(Cells * Cells * 6 + 3);
	for (int32 iy = 0; iy < W; ++iy)
	{
		for (int32 ix = 0; ix < W; ++ix)
		{
			V.Add(FVector((ix * StepM - Half) * 100.0, (iy * StepM - Half) * 100.0, 0.0));
			Nrm.Add(FVector::UpVector);
		}
	}
	for (int32 iy = 0; iy < Cells; ++iy)
	{
		for (int32 ix = 0; ix < Cells; ++ix)
		{
			const int32 A = iy * W + ix, B = A + 1, C = A + W, D = C + 1;
			Tri.Append({ A, C, B, B, C, D });
		}
	}
	// 頂点は z = 0 の平面なので、波の変位（WPO）でカリングされないよう ±50 m の縮退三角形で外形を広げる
	const int32 Mid = (W / 2) * W + W / 2;
	V.Add(FVector(V[Mid].X, V[Mid].Y, 5000.0));
	V.Add(FVector(V[Mid].X, V[Mid].Y, -5000.0));
	Nrm.Add(FVector::UpVector);
	Nrm.Add(FVector::UpVector);
	Tri.Append({ Mid, V.Num() - 2, V.Num() - 1 });
	Near->CreateMeshSection(0, V, Tri, Nrm, TArray<FVector2D>(), TArray<FColor>(), TArray<FProcMeshTangent>(), false);

	Far->SetRelativeLocation(FVector(0.0, 0.0, -5.0));
	Far->SetRelativeScale3D(FVector(FarSizeM, FarSizeM, 1.0));   // エンジンの Plane は 1 m 四方

	if (NearMaterial)
	{
		NearMid = UMaterialInstanceDynamic::Create(NearMaterial, this);
		NearMid->SetTextureParameterValue(TEXT("WaveTex"), WaveTex);
		NearMid->SetScalarParameterValue(TEXT("FadeStart"), FadeStartM);
		NearMid->SetScalarParameterValue(TEXT("FadeEnd"), FadeEndM);
		NearMid->SetScalarParameterValue(TEXT("HorizDisp"), HorizontalDisplacement);
		Near->SetMaterial(0, NearMid);
	}
	if (FarMaterial)
	{
		FarMid = UMaterialInstanceDynamic::Create(FarMaterial, this);
		FarMid->SetScalarParameterValue(TEXT("Hole"), HoleRadiusM);
		FarMid->SetTextureParameterValue(TEXT("WaveTex"), WaveTex);
		Far->SetMaterial(0, FarMid);
	}
	UE_LOG(LogShipOcean, Log, TEXT("ocean mesh %.0f m / %d cells (%.2f m), far %.0f m, fade %.0f..%.0f m, hole %.0f m"),
		MeshSizeM, Cells, StepM, FarSizeM, FadeStartM, FadeEndM, HoleRadiusM);
}

void AShipOcean::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (bSurfaceCheckRequested && bRendered)
	{
		// 前のフレームで描画へ渡した状態（位置・波の表）がいま描かれている。新しい状態を渡す前に読む
		bSurfaceCheckRequested = false;
		RunSurfaceCheck();
	}
	if (!bPhysicsTime)
	{
		TimeSec = GetWorld()->GetTimeSeconds();
	}
	// メッシュの中心をカメラの真下へ（格子の間隔でそろえる）
	if (APlayerController* PC = GetWorld()->GetFirstPlayerController(); PC && PC->PlayerCameraManager)
	{
		const FVector Cam = ShipFrame::UnrealToLocal(PC->PlayerCameraManager->GetCameraLocation());
		CenterEnu = FVector2D(FMath::RoundToDouble(Cam.X / StepM) * StepM, FMath::RoundToDouble(Cam.Y / StepM) * StepM);
	}
	SetActorLocation(ShipFrame::LocalToUnreal(CenterEnu.X, CenterEnu.Y, 0.0));
	UpdateGpu();
	if (bAutoSurfaceCheck && TimeSec >= NextAutoCheck)
	{
		NextAutoCheck = TimeSec + 2.0;
		RequestSurfaceCheck();
	}
}

void AShipOcean::UpdateGpu()
{
	if (!WaveTex || !NearMid)
	{
		return;
	}
	using namespace ShipOceanShader;
	TArray<FLinearColor>* Data = new TArray<FLinearColor>();
	Data->SetNumZeroed(TexWidth * 2);
	const double CE = CenterEnu.X, CN = CenterEnu.Y, T = TimeSec;
	const int32 N = ComponentCount();
	const auto& Comps = Engine.get_components();
	for (int32 i = 0; i < N; ++i)
	{
		const auto& C = Comps[i];
		const double Q = C.k > 1e-9 ? C.choppiness * C.amp / C.k : 0.0;
		(*Data)[i] = FLinearColor(C.kx, C.ky, C.amp, Q);
		(*Data)[TexWidth + i] = FLinearColor(Wrap2Pi(C.phase - C.omega * T + C.kx * CE + C.ky * CN), 0.0f, 0.0f, 0.0f);
	}
	for (int32 j = 0; j < Ripples.Num(); ++j)
	{
		const FRipple& R = Ripples[j];
		(*Data)[MaxWaves + j] = FLinearColor(R.Kx, R.Ky, R.Amp * RippleStrength, 0.0f);
		(*Data)[TexWidth + MaxWaves + j] = FLinearColor(Wrap2Pi(R.Phase - R.Omega * T + R.Kx * CE + R.Ky * CN), 0.0f, 0.0f, 0.0f);
	}
	FUpdateTextureRegion2D* Region = new FUpdateTextureRegion2D(0, 0, 0, 0, TexWidth, 2);
	WaveTex->UpdateTextureRegions(0, 1, Region, TexWidth * sizeof(FLinearColor), sizeof(FLinearColor), reinterpret_cast<uint8*>(Data->GetData()),
		[Data](uint8*, const FUpdateTextureRegion2D* R) { delete Data; delete R; });
	NearMid->SetScalarParameterValue(TEXT("WaveCount"), N);
	NearMid->SetScalarParameterValue(TEXT("RippleCount"), Ripples.Num());
	NearMid->SetScalarParameterValue(TEXT("Tide"), Params.tide_level_m);
	if (FarMid)
	{
		FarMid->SetScalarParameterValue(TEXT("WaveCount"), N);
	}
	RenderedTime = T;
	RenderedCenter = CenterEnu;
	bRendered = true;
}

void AShipOcean::RunSurfaceCheck()
{
	if (!Capture)
	{
		CaptureTarget = NewObject<UTextureRenderTarget2D>(this);
		CaptureTarget->RenderTargetFormat = RTF_RGBA32f;
		CaptureTarget->ClearColor = FLinearColor(1e9f, 0.0f, 0.0f, 0.0f);
		CaptureTarget->InitAutoFormat(CheckPixels, CheckPixels);
		CaptureTarget->UpdateResourceImmediate(true);
		Capture = NewObject<USceneCaptureComponent2D>(this);
		Capture->SetUsingAbsoluteLocation(true);
		Capture->SetUsingAbsoluteRotation(true);
		Capture->ProjectionType = ECameraProjectionMode::Orthographic;
		Capture->OrthoWidth = CheckPixels * StepM * 100.0;
		Capture->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
		Capture->bCaptureEveryFrame = false;
		Capture->bCaptureOnMovement = false;
		Capture->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
		Capture->ShowOnlyComponents.Add(Near);
		Capture->TextureTarget = CaptureTarget;
		Capture->RegisterComponent();
	}
	// 画素の中心が格子の頂点に乗るよう、カメラを半画素ずらす。真下を見るカメラ：画面の上 = +X、右 = +Y
	const double S = StepM * 100.0;
	const FVector Cam = ShipFrame::LocalToUnreal(RenderedCenter.X, RenderedCenter.Y, CheckHeightM) + FVector(0.5 * S, -0.5 * S, 0.0);
	Capture->SetWorldLocationAndRotation(Cam, FRotator(-90.0, 0.0, 0.0));
	Capture->CaptureScene();
	TArray<FLinearColor> Px;
	if (!CaptureTarget->GameThread_GetRenderTargetResource()->ReadLinearColorPixels(Px) || Px.Num() != CheckPixels * CheckPixels)
	{
		UE_LOG(LogShipOcean, Error, TEXT("surface check: cannot read the capture"));
		++SurfaceMisses;
		return;
	}
	const double Tide = Params.tide_level_m;
	double MaxErr = 0.0, MaxEta = 0.0;
	int64 Misses = 0;
	for (int32 py = 0; py < CheckPixels; ++py)
	{
		for (int32 px = 0; px < CheckPixels; ++px)
		{
			const double X = Cam.X + (CheckPixels * 0.5 - py - 0.5) * S;
			const double Y = Cam.Y + (px + 0.5 - CheckPixels * 0.5) * S;
			const FVector L = ShipFrame::UnrealToLocal(FVector(X, Y, 0.0));
			const double Depth = Px[py * CheckPixels + px].R;
			if (!(Depth > 0.0 && Depth < 2.0 * CheckHeightM * 100.0))
			{
				++Misses;
				continue;
			}
			const double Measured = CheckHeightM - Depth / 100.0;
			const double Dist = FVector2D(L.X - RenderedCenter.X, L.Y - RenderedCenter.Y).Size();
			const double Fade = 1.0 - FMath::SmoothStep(FadeStartM, FadeEndM, Dist);
			const double Want = Tide + (ElevationAt(L.X, L.Y, RenderedTime) - Tide) * Fade;
			MaxErr = FMath::Max(MaxErr, FMath::Abs(Measured - Want));
			MaxEta = FMath::Max(MaxEta, FMath::Abs(Want - Tide));
		}
	}
	++SurfaceChecks;
	SurfacePoints += CheckPixels * CheckPixels - Misses;
	SurfaceMisses += Misses;
	SurfaceMaxErr = FMath::Max(SurfaceMaxErr, MaxErr);
	SurfaceMaxEta = FMath::Max(SurfaceMaxEta, MaxEta);
	UE_LOG(LogShipOcean, Log, TEXT("SHIP_SEA surface check %d: t=%.3f s center (%.1f, %.1f) m, %d points, misses %lld, max err %.2e m (tol %.0e), max |eta| %.2f m"),
		SurfaceChecks, RenderedTime, RenderedCenter.X, RenderedCenter.Y, CheckPixels * CheckPixels - static_cast<int32>(Misses), Misses, MaxErr, SurfaceTolM, MaxEta);
}

bool AShipOcean::RunParity(const FString& Path)
{
	// Godot の tests/unit/wave_parity と同じ比べ方（相対 1e-12 / 絶対 1e-12）。参照は Python 実装（C++ とビット一致を確認済み）の値
	FString Text;
	TArray<TSharedPtr<FJsonValue>> Cases;
	if (!FFileHelper::LoadFileToString(Text, *Path) || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Cases) || Cases.Num() == 0)
	{
		UE_LOG(LogShipOcean, Error, TEXT("HAKONIWA_UNREAL_SHIP_WAVE_PARITY_FAILED reference not readable: %s"), *Path);
		return false;
	}
	double MaxErr = 0.0;
	int32 Checked = 0;
	TArray<FString> Problems;
	auto Compare = [&](const FString& Label, double Actual, double Expected)
	{
		const double Err = FMath::Abs(Actual - Expected);
		MaxErr = FMath::Max(MaxErr, Err);
		++Checked;
		if (Err > FMath::Max(1e-12, 1e-12 * FMath::Abs(Expected)) && Problems.Num() < 10)
		{
			Problems.Add(FString::Printf(TEXT("%s: %.17g expected %.17g"), *Label, Actual, Expected));
		}
	};
	for (int32 ci = 0; ci < Cases.Num(); ++ci)
	{
		const TSharedPtr<FJsonObject> Case = Cases[ci]->AsObject();
		const TSharedPtr<FJsonObject> P = Case->GetObjectField(TEXT("params"));
		ShipWave::FParams W;
		W.hs_m = P->GetNumberField(TEXT("hs_m"));
		W.tp_s = P->GetNumberField(TEXT("tp_s"));
		W.wave_dir_rad = P->GetNumberField(TEXT("wave_dir_rad"));
		W.gamma = P->GetNumberField(TEXT("gamma"));
		W.seed = static_cast<uint32_t>(P->GetNumberField(TEXT("seed")));
		W.water_depth_m = P->GetNumberField(TEXT("water_depth_m"));
		W.tide_level_m = P->GetNumberField(TEXT("tide_level_m"));
		W.num_components = P->GetIntegerField(TEXT("num_components"));
		ShipWave::FEngine E;
		E.initialize(W);
		const TArray<TSharedPtr<FJsonValue>>& Comps = Case->GetArrayField(TEXT("components"));
		if (static_cast<int32>(E.get_components().size()) != Comps.Num())
		{
			Problems.Add(FString::Printf(TEXT("case%d: %d components, expected %d"), ci, static_cast<int32>(E.get_components().size()), Comps.Num()));
			continue;
		}
		for (int32 i = 0; i < Comps.Num(); ++i)
		{
			const TSharedPtr<FJsonObject> C = Comps[i]->AsObject();
			const auto& A = E.get_components()[i];
			const FString L = FString::Printf(TEXT("case%d comp%d "), ci, i);
			Compare(L + TEXT("omega"), A.omega, C->GetNumberField(TEXT("omega")));
			Compare(L + TEXT("k"), A.k, C->GetNumberField(TEXT("k")));
			Compare(L + TEXT("kx"), A.kx, C->GetNumberField(TEXT("kx")));
			Compare(L + TEXT("ky"), A.ky, C->GetNumberField(TEXT("ky")));
			Compare(L + TEXT("amp"), A.amp, C->GetNumberField(TEXT("amp")));
			Compare(L + TEXT("phase"), A.phase, C->GetNumberField(TEXT("phase")));
			Compare(L + TEXT("dir_rad"), A.dir_rad, C->GetNumberField(TEXT("dir_rad")));
			Compare(L + TEXT("choppiness"), A.choppiness, C->GetNumberField(TEXT("choppiness")));
		}
		for (const TSharedPtr<FJsonValue>& SV : Case->GetArrayField(TEXT("samples")))
		{
			const TSharedPtr<FJsonObject> S = SV->AsObject();
			const double X = S->GetNumberField(TEXT("x")), Y = S->GetNumberField(TEXT("y")), T = S->GetNumberField(TEXT("t"));
			const auto Surf = E.get_surface(X, Y, T);
			const FString L = FString::Printf(TEXT("case%d (%.1f,%.1f,t=%.1f) "), ci, X, Y, T);
			Compare(L + TEXT("elevation"), Surf.elevation, S->GetNumberField(TEXT("elevation")));
			Compare(L + TEXT("slope_x"), Surf.slope_x, S->GetNumberField(TEXT("slope_x")));
			Compare(L + TEXT("slope_y"), Surf.slope_y, S->GetNumberField(TEXT("slope_y")));
			Compare(L + TEXT("disp_x"), Surf.disp_x, S->GetNumberField(TEXT("disp_x")));
			Compare(L + TEXT("disp_y"), Surf.disp_y, S->GetNumberField(TEXT("disp_y")));
			const TArray<TSharedPtr<FJsonValue>>& Nv = S->GetArrayField(TEXT("normal"));
			for (int32 a = 0; a < 3; ++a)
			{
				Compare(L + FString::Printf(TEXT("normal[%d]"), a), Surf.normal[a], Nv[a]->AsNumber());
			}
			Compare(L + TEXT("get_elevation"), E.get_elevation(X, Y, T), S->GetNumberField(TEXT("elevation")));
		}
	}
	if (Problems.Num() > 0)
	{
		UE_LOG(LogShipOcean, Error, TEXT("HAKONIWA_UNREAL_SHIP_WAVE_PARITY_FAILED %s"), *FString::Join(Problems, TEXT("; ")));
		return false;
	}
	UE_LOG(LogShipOcean, Display, TEXT("HAKONIWA_UNREAL_SHIP_WAVE_PARITY_OK cases=%d values=%d max_abs_err=%.3e"), Cases.Num(), Checked, MaxErr);
	return true;
}
