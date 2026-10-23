// HakoniwaShipEditor.cpp
// 海の場面を組む入口（SHP-24）。hakoniwa-unreal-rover の HakoniwaRoverEditor と同じ形。
//   自動：UnrealEditor.exe <uproject> -ShipBuildScene [-ShipMap=/Game/...] [-ShipQuit]
//   手で：エディタのコンソールで  Ship.BuildScene [-ShipMap=/Game/...]
//
// 場面（Step 3）：江の島の Godot シーン（D:\work_godot\3DMode_enoshima_PC）にならい、
//   空    ：Poly Haven の空パノラマ（SourceArt/Sky/sky_2k.exr、tools/make_sky_hdri.py）を空の球に貼り、空の光はそれを実時間で取り込む
//   太陽  ：パノラマの太陽の向き（SourceArt/Sky/sky.json）の平行光。照度と空の輝度の比はパノラマのまま
//   霧    ：水平線の空の色の高さの霧（遠くの海を空になじませる）
//   船    ：Godot ビューアと同じ glb を部品ごとの StaticMesh に（ShipModelImport、Step 4）
//   海面  ：AShipOcean（物理と同じ決定論的 Gerstner 波面、D4・D19）。材質は ShipOceanShader.h の HLSL から作る
//   観客の開始位置・船団（AShipFleet、Step 2）
// 作る資産は /Game/Ship/Generated（材質・空の画像・船のメッシュ）と地図。どちらも生成物（git に入れない）。

#include "AssetImportTask.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "AssetToolsModule.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Containers/Ticker.h"
#include "Dom/JsonObject.h"
#include "Editor.h"
#include "Engine/DirectionalLight.h"
#include "Engine/ExponentialHeightFog.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/SkyLight.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/Texture2D.h"
#include "FileHelpers.h"
#include "GameFramework/PlayerStart.h"
#include "GameFramework/WorldSettings.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformMisc.h"
#include "MaterialEditingLibrary.h"
#include "Materials/Material.h"
#include "Materials/MaterialExpressionCameraPositionWS.h"
#include "Materials/MaterialExpressionCameraVectorWS.h"
#include "Materials/MaterialExpressionTransform.h"
#include "Materials/MaterialExpressionComponentMask.h"
#include "Materials/MaterialExpressionConstant.h"
#include "Materials/MaterialExpressionCustom.h"
#include "Materials/MaterialExpressionMultiply.h"
#include "Materials/MaterialExpressionObjectPositionWS.h"
#include "Materials/MaterialExpressionPixelDepth.h"
#include "Materials/MaterialExpressionScalarParameter.h"
#include "Materials/MaterialExpressionSubtract.h"
#include "Materials/MaterialExpressionTextureObjectParameter.h"
#include "Materials/MaterialExpressionTextureSampleParameter2D.h"
#include "Materials/MaterialExpressionVectorParameter.h"
#include "Materials/MaterialExpressionWorldPosition.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "ShipFleet.h"
#include "ShipFrame.h"
#include "ShipLighting.h"
#include "ShipOcean.h"
#include "ShipOceanShader.h"
#include "ShipModelImport.h"
#include "ShipViewGameMode.h"

DEFINE_LOG_CATEGORY_STATIC(LogShipEditor, Log, All);

namespace
{
	const TCHAR* DefaultMap = TEXT("/Game/Ship/Levels/ShipSea");
	const TCHAR* GeneratedDir = TEXT("/Game/Ship/Generated");
	// 空の画像の値 → Unreal の輝度 [cd/m²]・照度 [lx]。太陽の照度がおよそ 9 万 lx（晴れの昼）になる倍率
	constexpr double SkyNitsPerUnit = 20000.0;
	// 露出 [EV100]（晴天の昼。r.EyeAdaptation.CachedLightingPreExposure=8 の範囲 −4〜16 の内）
	constexpr float DaylightEV100 = 15.5f;   // 14.5・15.5・16 を比べて決めた（-ShipExposure=）
	// 江の島の海（enoshima.tscn の ShaderMaterial_ocean の deep_color / shallow_color、sRGB）を線形にした色
	const FLinearColor DeepColor = FLinearColor::FromSRGBColor(FColor(18, 38, 55));
	const FLinearColor ShallowColor = FLinearColor::FromSRGBColor(FColor(23, 71, 74));

	FString ParseMap(const TCHAR* Cmd)
	{
		FString Map = DefaultMap;
		FParse::Value(Cmd, TEXT("-ShipMap="), Map);
		return Map;
	}

	/** /Game/Ship/Generated/<Name> の材質。あれば式をすべて消して作り直す（参照は保たれる） */
	UMaterial* MakeMaterial(const FString& Name)
	{
		const FString PackageName = FString(GeneratedDir) / Name;
		UMaterial* M = LoadObject<UMaterial>(nullptr, *(PackageName + TEXT(".") + Name), nullptr, LOAD_NoWarn | LOAD_Quiet);
		if (M)
		{
			UMaterialEditingLibrary::DeleteAllMaterialExpressions(M);
		}
		else
		{
			UPackage* Package = CreatePackage(*PackageName);
			M = NewObject<UMaterial>(Package, *Name, RF_Public | RF_Standalone | RF_Transactional);
			FAssetRegistryModule::AssetCreated(M);
		}
		M->Modify();
		return M;
	}

	template <typename T>
	T* Add(UMaterial* M, int32 X, int32 Y)
	{
		return Cast<T>(UMaterialEditingLibrary::CreateMaterialExpression(M, T::StaticClass(), X, Y));
	}

	UMaterialExpressionScalarParameter* Scalar(UMaterial* M, const TCHAR* Name, float Value, int32 Y)
	{
		auto* P = Add<UMaterialExpressionScalarParameter>(M, -900, Y);
		P->ParameterName = Name;
		P->DefaultValue = Value;
		return P;
	}

	UMaterialExpressionVectorParameter* Vector(UMaterial* M, const TCHAR* Name, const FLinearColor& Value, int32 Y)
	{
		auto* P = Add<UMaterialExpressionVectorParameter>(M, -900, Y);
		P->ParameterName = Name;
		P->DefaultValue = Value;
		return P;
	}

	/** Custom 式。Inputs は (名前, 式) の組 */
	UMaterialExpressionCustom* Custom(UMaterial* M, const TCHAR* Desc, const FString& Code, ECustomMaterialOutputType Out,
		const TArray<TPair<FString, UMaterialExpression*>>& Inputs, int32 X, int32 Y)
	{
		auto* C = Add<UMaterialExpressionCustom>(M, X, Y);
		C->Description = Desc;
		C->Code = Code;
		C->OutputType = Out;
		C->Inputs.Reset();
		for (const auto& In : Inputs)
		{
			FCustomInput CI;
			CI.InputName = FName(*In.Key);
			CI.Input.Connect(0, In.Value);
			C->Inputs.Add(CI);
		}
		return C;
	}

	UMaterialExpressionComponentMask* Mask(UMaterial* M, UMaterialExpression* From, bool R, bool G, bool B, bool A, int32 X, int32 Y)
	{
		auto* K = Add<UMaterialExpressionComponentMask>(M, X, Y);
		K->R = R; K->G = G; K->B = B; K->A = A;
		K->Input.Connect(0, From);
		return K;
	}

	/** メッシュの中心からの相対位置 [cm]：WorldPosition（材質の変位を除く）− ObjectPosition */
	UMaterialExpression* RelPosition(UMaterial* M)
	{
		auto* Wp = Add<UMaterialExpressionWorldPosition>(M, -1200, -400);
		Wp->WorldPositionShaderOffset = WPT_ExcludeAllShaderOffsets;
		auto* Op = Add<UMaterialExpressionObjectPositionWS>(M, -1200, -300);
		auto* Sub = Add<UMaterialExpressionSubtract>(M, -1000, -350);
		Sub->A.Connect(0, Wp);
		Sub->B.Connect(0, Op);
		return Sub;
	}

	bool Finish(UMaterial* M)
	{
		UMaterialEditingLibrary::LayoutMaterialExpressions(M);
		UMaterialEditingLibrary::RecompileMaterial(M);
		M->MarkPackageDirty();
		UE_LOG(LogShipEditor, Log, TEXT("material %s: %d expressions"), *M->GetPathName(), M->GetExpressions().Num());
		return true;
	}

	/** 波面メッシュの材質（ShipOceanShader の WPO・画素の式） */
	UMaterial* BuildOceanMaterial()
	{
		using namespace ShipOceanShader;
		UMaterial* M = MakeMaterial(TEXT("M_ShipOcean"));
		M->BlendMode = BLEND_Opaque;
		M->SetShadingModel(MSM_DefaultLit);
		M->bTangentSpaceNormal = false;   // 世界の法線を直接出す
		UMaterialExpression* Rel = RelPosition(M);
		auto* Tex = Add<UMaterialExpressionTextureObjectParameter>(M, -900, 0);
		Tex->ParameterName = TEXT("WaveTex");
		Tex->Texture = LoadObject<UTexture>(nullptr, TEXT("/Engine/EngineResources/Black.Black"));
		auto* Count = Scalar(M, TEXT("WaveCount"), 0.0f, 100);
		auto* Ripple = Scalar(M, TEXT("RippleCount"), 0.0f, 200);
		auto* Tide = Scalar(M, TEXT("Tide"), 0.0f, 300);
		auto* FadeStart = Scalar(M, TEXT("FadeStart"), 950.0f, 400);
		auto* FadeEnd = Scalar(M, TEXT("FadeEnd"), 1180.0f, 500);
		auto* HorizDisp = Scalar(M, TEXT("HorizDisp"), 0.0f, 600);
		auto* Depth = Add<UMaterialExpressionPixelDepth>(M, -900, 700);
		auto* Wpo = Custom(M, TEXT("ShipWaveWPO"), WpoCode(), CMOT_Float3,
			{ { TEXT("Rel"), Rel }, { TEXT("WaveTex"), Tex }, { TEXT("WaveCount"), Count }, { TEXT("Tide"), Tide },
			  { TEXT("FadeStart"), FadeStart }, { TEXT("FadeEnd"), FadeEnd }, { TEXT("HorizDisp"), HorizDisp } }, -500, 0);
		const FString Pixel = FString(PixelCode()).Replace(TEXT("MAX_WAVES"), *FString::FromInt(MaxWaves));
		auto* Px = Custom(M, TEXT("ShipWavePixel"), Pixel, CMOT_Float4,
			{ { TEXT("Rel"), Rel }, { TEXT("WaveTex"), Tex }, { TEXT("WaveCount"), Count }, { TEXT("RippleCount"), Ripple },
			  { TEXT("FadeStart"), FadeStart }, { TEXT("FadeEnd"), FadeEnd }, { TEXT("Depth"), Depth } }, -500, 300);
		auto* Normal = Mask(M, Px, true, true, true, false, -250, 300);
		auto* Height = Mask(M, Px, false, false, false, true, -250, 400);
		auto* Color = Custom(M, TEXT("ShipWaterColor"), ColorCode(), CMOT_Float3,
			{ { TEXT("H"), Height }, { TEXT("Deep"), Vector(M, TEXT("DeepColor"), DeepColor, 800) },
			  { TEXT("Shallow"), Vector(M, TEXT("ShallowColor"), ShallowColor, 900) } }, -250, 500);
		UMaterialEditingLibrary::ConnectMaterialProperty(Wpo, FString(), MP_WorldPositionOffset);
		UMaterialEditingLibrary::ConnectMaterialProperty(Normal, FString(), MP_Normal);
		UMaterialEditingLibrary::ConnectMaterialProperty(Color, FString(), MP_BaseColor);
		auto* Rough = Custom(M, TEXT("ShipWaterRoughness"), RoughnessCode(), CMOT_Float1,
			{ { TEXT("Base"), Scalar(M, TEXT("Roughness"), 0.05f, 1000) }, { TEXT("Depth"), Depth } }, -250, 700);
		UMaterialEditingLibrary::ConnectMaterialProperty(Rough, FString(), MP_Roughness);
		UMaterialEditingLibrary::ConnectMaterialProperty(Scalar(M, TEXT("Specular"), 0.3f, 1100), FString(), MP_Specular);
		return Finish(M) ? M : nullptr;
	}

	/** 外側の平面（潮位面）の材質：内側は描かない。高さは平らで、傾き（法線）・色・粗さは波面メッシュと同じ式（継ぎ目を目立たせない） */
	UMaterial* BuildFarMaterial()
	{
		using namespace ShipOceanShader;
		UMaterial* M = MakeMaterial(TEXT("M_ShipOceanFar"));
		M->BlendMode = BLEND_Masked;
		M->SetShadingModel(MSM_DefaultLit);
		M->bTangentSpaceNormal = false;
		UMaterialExpression* Rel = RelPosition(M);
		auto* Hole = Scalar(M, TEXT("Hole"), 1150.0f, 0);
		auto* Opacity = Custom(M, TEXT("ShipFarMask"), FarMaskCode(), CMOT_Float1,
			{ { TEXT("Rel"), Rel }, { TEXT("Hole"), Hole } }, -500, 0);
		auto* Tex = Add<UMaterialExpressionTextureObjectParameter>(M, -900, 100);
		Tex->ParameterName = TEXT("WaveTex");
		Tex->Texture = LoadObject<UTexture>(nullptr, TEXT("/Engine/EngineResources/Black.Black"));
		auto* Count = Scalar(M, TEXT("WaveCount"), 0.0f, 200);
		auto* NoRipple = Scalar(M, TEXT("RippleCount"), 0.0f, 300);
		auto* NoFade = Scalar(M, TEXT("FadeStart"), 1.0e9f, 400);
		auto* NoFadeEnd = Scalar(M, TEXT("FadeEnd"), 2.0e9f, 500);
		auto* Depth = Add<UMaterialExpressionPixelDepth>(M, -900, 600);
		const FString Pixel = FString(PixelCode()).Replace(TEXT("MAX_WAVES"), *FString::FromInt(MaxWaves));
		auto* Px = Custom(M, TEXT("ShipWavePixel"), Pixel, CMOT_Float4,
			{ { TEXT("Rel"), Rel }, { TEXT("WaveTex"), Tex }, { TEXT("WaveCount"), Count }, { TEXT("RippleCount"), NoRipple },
			  { TEXT("FadeStart"), NoFade }, { TEXT("FadeEnd"), NoFadeEnd }, { TEXT("Depth"), Depth } }, -500, 300);
		auto* Normal = Mask(M, Px, true, true, true, false, -250, 300);
		auto* Height = Mask(M, Px, false, false, false, true, -250, 400);
		auto* Color = Custom(M, TEXT("ShipWaterColor"), ColorCode(), CMOT_Float3,
			{ { TEXT("H"), Height }, { TEXT("Deep"), Vector(M, TEXT("DeepColor"), DeepColor, 800) },
			  { TEXT("Shallow"), Vector(M, TEXT("ShallowColor"), ShallowColor, 900) } }, -250, 500);
		auto* Rough = Custom(M, TEXT("ShipWaterRoughness"), RoughnessCode(), CMOT_Float1,
			{ { TEXT("Base"), Scalar(M, TEXT("Roughness"), 0.05f, 1000) }, { TEXT("Depth"), Depth } }, -250, 700);
		UMaterialEditingLibrary::ConnectMaterialProperty(Opacity, FString(), MP_OpacityMask);
		UMaterialEditingLibrary::ConnectMaterialProperty(Normal, FString(), MP_Normal);
		UMaterialEditingLibrary::ConnectMaterialProperty(Color, FString(), MP_BaseColor);
		UMaterialEditingLibrary::ConnectMaterialProperty(Rough, FString(), MP_Roughness);
		UMaterialEditingLibrary::ConnectMaterialProperty(Scalar(M, TEXT("Specular"), 0.3f, 1100), FString(), MP_Specular);
		return Finish(M) ? M : nullptr;
	}

	/** 船の塗装の材質（Step 4）：glb の材質（両面、色・金属度・粗さ・発光）。値は実行時に AShipActor が材質ごとに入れる */
	UMaterial* BuildPaintMaterial()
	{
		UMaterial* M = MakeMaterial(TEXT("M_ShipPaint"));
		M->SetShadingModel(MSM_DefaultLit);
		M->TwoSided = true;   // glb の材質はすべて doubleSided（Godot と同じ）
		UMaterialEditingLibrary::ConnectMaterialProperty(Vector(M, TEXT("BaseColor"), FLinearColor(0.6f, 0.6f, 0.6f), 0), FString(), MP_BaseColor);
		UMaterialEditingLibrary::ConnectMaterialProperty(Scalar(M, TEXT("Metallic"), 0.0f, 100), FString(), MP_Metallic);
		UMaterialEditingLibrary::ConnectMaterialProperty(Scalar(M, TEXT("Roughness"), 0.5f, 200), FString(), MP_Roughness);
		auto* Mul = Add<UMaterialExpressionMultiply>(M, -500, 300);
		Mul->A.Connect(0, Vector(M, TEXT("Emissive"), FLinearColor::Black, 300));
		Mul->B.Connect(0, Scalar(M, TEXT("EmissiveNits"), 1000.0f, 400));
		// 航海灯の射光範囲（Step 5）：灯火から視点への向きを部品の座標に移し、方位が範囲の外なら発光しない（既定 Half = 180 は全周）
		auto* CamPos = Add<UMaterialExpressionCameraPositionWS>(M, -1300, 500);
		auto* ObjPos = Add<UMaterialExpressionObjectPositionWS>(M, -1300, 600);
		auto* ToCam = Add<UMaterialExpressionSubtract>(M, -1100, 550);
		ToCam->A.Connect(0, CamPos);
		ToCam->B.Connect(0, ObjPos);
		auto* Local = Add<UMaterialExpressionTransform>(M, -900, 550);
		Local->TransformSourceType = TRANSFORMSOURCE_World;
		Local->TransformType = TRANSFORM_Local;
		Local->Input.Connect(0, ToCam);
		auto* Sector = Custom(M, TEXT("ShipNavSector"), ShipOceanShader::NavSectorCode(), CMOT_Float1,
			{ { TEXT("Local"), Local }, { TEXT("Center"), Scalar(M, TEXT("SectorCenter"), 0.0f, 700) }, { TEXT("Half"), Scalar(M, TEXT("SectorHalf"), 180.0f, 800) } }, -600, 550);
		auto* Lit = Add<UMaterialExpressionMultiply>(M, -300, 400);
		Lit->A.Connect(0, Mul);
		Lit->B.Connect(0, Sector);
		UMaterialEditingLibrary::ConnectMaterialProperty(Lit, FString(), MP_EmissiveColor);
		return Finish(M) ? M : nullptr;
	}

	/** 空の画像（SourceArt/Sky/sky_2k.exr）を取り込む。正距円筒の 2D の HDR 画像として使う */
	UTexture2D* ImportSky()
	{
		const FString File = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() / TEXT("SourceArt/Sky/sky_2k.exr"));
		UAssetImportTask* Task = NewObject<UAssetImportTask>();
		Task->Filename = File;
		Task->DestinationPath = GeneratedDir;
		Task->DestinationName = TEXT("T_Sky");
		Task->bAutomated = true;
		Task->bReplaceExisting = true;
		Task->bSave = false;
		FAssetToolsModule::GetModule().Get().ImportAssetTasks({ Task });
		UTexture2D* Tex = nullptr;
		for (UObject* O : Task->GetObjects())
		{
			UE_LOG(LogShipEditor, Log, TEXT("imported %s -> %s (%s)"), *File, *O->GetPathName(), *O->GetClass()->GetName());
			Tex = Tex ? Tex : Cast<UTexture2D>(O);
		}
		if (!Tex)
		{
			UE_LOG(LogShipEditor, Error, TEXT("sky texture not imported as Texture2D: %s"), *File);
			return nullptr;
		}
		Tex->Modify();
		Tex->CompressionSettings = TC_HDR_Compressed;
		Tex->SRGB = false;
		Tex->MipGenSettings = TMGS_NoMipmaps;
		Tex->LODGroup = TEXTUREGROUP_Skybox;
		Tex->AddressX = TA_Wrap;
		Tex->AddressY = TA_Clamp;
		Tex->PostEditChange();
		Tex->MarkPackageDirty();
		return Tex;
	}

	/** 空の球の材質：視線の向きで正距円筒の画像を引く（光らない・空として空の光に取り込まれる） */
	UMaterial* BuildSkyMaterial(UTexture2D* SkyTex)
	{
		UMaterial* M = MakeMaterial(TEXT("M_ShipSky"));
		M->SetShadingModel(MSM_Unlit);
		M->TwoSided = true;
		M->bIsSky = true;
		auto* Cam = Add<UMaterialExpressionCameraVectorWS>(M, -900, 0);
		auto* Uv = Custom(M, TEXT("ShipSkyUv"), ShipOceanShader::SkyUvCode(), CMOT_Float2, { { TEXT("CamVec"), Cam } }, -600, 0);
		auto* Sample = Add<UMaterialExpressionTextureSampleParameter2D>(M, -350, 0);
		Sample->ParameterName = TEXT("SkyTex");
		Sample->Texture = SkyTex;
		Sample->SamplerType = SAMPLERTYPE_LinearColor;
		Sample->MipValueMode = TMVM_MipLevel;   // 経度 0/360° の継ぎ目で詳細度が飛ばないよう、いつも最も細かい画像
		Sample->Coordinates.Connect(0, Uv);
		auto* Zero = Add<UMaterialExpressionConstant>(M, -600, 150);
		Zero->R = 0.0f;
		Sample->MipValue.Connect(0, Zero);
		// 昼夜（Step 5）：AShipLighting がパラメータを入れる（太陽・月の円盤、夜の暗さと青み、星）
		auto* Rgb = Mask(M, Sample, true, true, true, false, -200, 0);
		auto* Color = Custom(M, TEXT("ShipSkyColor"), ShipOceanShader::SkyColorCode(), CMOT_Float3,
			{ { TEXT("Col"), Rgb }, { TEXT("CamVec"), Cam },
			  { TEXT("SkyNits"), Scalar(M, TEXT("SkyNits"), SkyNitsPerUnit, 200) },
			  { TEXT("Tint"), Vector(M, TEXT("Tint"), FLinearColor::White, 300) },
			  { TEXT("DiskDir"), Vector(M, TEXT("DiskDir"), FLinearColor(0.0f, 0.0f, 1.0f), 400) },
			  { TEXT("DiskCos"), Scalar(M, TEXT("DiskCos"), 0.99998930f, 500) },
			  { TEXT("DiskNits"), Scalar(M, TEXT("DiskNits"), 0.0f, 600) },
			  { TEXT("DiskColor"), Vector(M, TEXT("DiskColor"), FLinearColor::White, 700) },
			  { TEXT("Stars"), Scalar(M, TEXT("Stars"), 0.0f, 800) } }, -100, 0);
		UMaterialEditingLibrary::ConnectMaterialProperty(Color, FString(), MP_EmissiveColor);
		return Finish(M) ? M : nullptr;
	}

	TSharedPtr<FJsonObject> LoadSkyInfo()
	{
		FString Text;
		TSharedPtr<FJsonObject> Root;
		const FString File = FPaths::ProjectDir() / TEXT("SourceArt/Sky/sky.json");
		if (!FFileHelper::LoadFileToString(Text, *File) || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Root))
		{
			UE_LOG(LogShipEditor, Error, TEXT("not readable: %s"), *File);
			return nullptr;
		}
		return Root;
	}

	FVector JsonVec(const TSharedPtr<FJsonObject>& O, const TCHAR* Key)
	{
		const TArray<TSharedPtr<FJsonValue>>& A = O->GetArrayField(Key);
		return FVector(A[0]->AsNumber(), A[1]->AsNumber(), A[2]->AsNumber());
	}

	bool BuildScene()
	{
		const TSharedPtr<FJsonObject> SkyInfo = LoadSkyInfo();
		UTexture2D* SkyTex = ImportSky();
		UMaterial* OceanMat = BuildOceanMaterial();
		UMaterial* FarMat = BuildFarMaterial();
		UMaterial* SkyMat = SkyTex ? BuildSkyMaterial(SkyTex) : nullptr;
		UStaticMesh* Sphere = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
		// 船（Step 4）：glb の部品ごとの StaticMesh
		UMaterial* PaintMat = BuildPaintMaterial();
		const int32 ShipMeshes = PaintMat ? ShipModelImport::BuildShipMeshes(PaintMat) : -1;
		if (!SkyInfo || !OceanMat || !FarMat || !SkyMat || !Sphere || ShipMeshes <= 0)
		{
			return false;
		}
		UWorld* World = GEditor ? GEditor->NewMap(false) : nullptr;
		if (!World)
		{
			return false;
		}
		World->GetWorldSettings()->DefaultGameMode = AShipViewGameMode::StaticClass();

		// 太陽：パノラマの太陽の向き（ENU）。光は太陽から来る向き。照度 = パノラマの太陽の放射照度 × 空の倍率
		// ★ ADirectionalLight は生成時に光へピッチ −46° を入れていて、スポーンの回転に足される（rover の README）。回転は生成のあとで決める
		const FVector SunEnu = JsonVec(SkyInfo, TEXT("sun_dir_enu"));
		const FVector SunRgb = JsonVec(SkyInfo, TEXT("sun_rgb_norm"));
		const double SunLux = SkyInfo->GetNumberField(TEXT("sun_irradiance")) * SkyNitsPerUnit;
		ADirectionalLight* Sun = World->SpawnActor<ADirectionalLight>();
		Sun->SetActorRotation((-ShipFrame::LocalDirToUnreal(SunEnu.X, SunEnu.Y, SunEnu.Z)).Rotation());
		Sun->SetActorLabel(TEXT("Sun"));
		UDirectionalLightComponent* SunC = CastChecked<UDirectionalLightComponent>(Sun->GetLightComponent());
		SunC->SetMobility(EComponentMobility::Movable);
		SunC->SetIntensity(SunLux);
		SunC->SetLightColor(FLinearColor(SunRgb.X, SunRgb.Y, SunRgb.Z));
		SunC->SetCastShadows(true);

		// 空の球（半径 100 km。材質は向きだけで引くので位置は問わない）
		AStaticMeshActor* SkySphere = World->SpawnActor<AStaticMeshActor>();
		SkySphere->SetActorLabel(TEXT("SkySphere"));
		UStaticMeshComponent* SkyC = SkySphere->GetStaticMeshComponent();
		SkyC->SetMobility(EComponentMobility::Movable);
		SkyC->SetStaticMesh(Sphere);
		SkyC->SetMaterial(0, SkyMat);
		SkyC->SetCastShadow(false);
		SkyC->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		SkyC->bAffectDistanceFieldLighting = false;
		SkyC->bAffectDynamicIndirectLighting = false;
		SkySphere->SetActorScale3D(FVector(200000.0));   // エンジンの Sphere は直径 1 m

		ASkyLight* SkyLight = World->SpawnActor<ASkyLight>();
		SkyLight->SetActorLabel(TEXT("SkyLight"));
		USkyLightComponent* SkyLightC = SkyLight->GetLightComponent();
		SkyLightC->SetMobility(EComponentMobility::Movable);
		SkyLightC->bRealTimeCapture = true;   // 空の球（bIsSky の材質）を取り込む
		SkyLightC->bLowerHemisphereIsBlack = false;   // パノラマの下半分（空の映り込み）も使う。波で下向きに反射した光が黒くならない

		// 月（Step 5、夜だけ点く。AShipLighting が向き・照度を入れる）
		ADirectionalLight* Moon = World->SpawnActor<ADirectionalLight>();
		Moon->SetActorLabel(TEXT("Moon"));
		UDirectionalLightComponent* MoonC = CastChecked<UDirectionalLightComponent>(Moon->GetLightComponent());
		MoonC->SetMobility(EComponentMobility::Movable);
		MoonC->SetIntensity(0.0f);
		MoonC->SetVisibility(false);

		// 霧：水平線の空の色
		const FVector Horizon = JsonVec(SkyInfo, TEXT("horizon_rgb")) * SkyNitsPerUnit;
		AExponentialHeightFog* Fog = World->SpawnActor<AExponentialHeightFog>();
		Fog->SetActorLabel(TEXT("HeightFog"));
		UExponentialHeightFogComponent* FogC = Fog->GetComponent();
		FogC->SetFogDensity(0.0015f);
		FogC->SetFogHeightFalloff(0.05f);
		FogC->SetFogInscatteringColor(FLinearColor(Horizon.X, Horizon.Y, Horizon.Z));
		FogC->SetFogCutoffDistance(9.5e6f);   // 空の球（100 km）には掛けない

		// 露出（Step 4）：自動露出は暗い海に合わせて EV 13 まで明るくし、船の塗装が白っぽく見えた（Godot の紺の船体が水色に）。
		// 晴天の昼の露出（sunny 16 の EV 15 付近）に固定する（EV 15.5 で Godot と同じく紺の船体・はっきりしたコンテナの色）
		APostProcessVolume* Post = World->SpawnActor<APostProcessVolume>();
		Post->SetActorLabel(TEXT("DaylightExposure"));
		Post->bUnbound = true;
		Post->Settings.bOverride_AutoExposureMinBrightness = true;
		Post->Settings.bOverride_AutoExposureMaxBrightness = true;
		Post->Settings.AutoExposureMinBrightness = DaylightEV100;
		Post->Settings.AutoExposureMaxBrightness = DaylightEV100;
		// 太陽光のまぶしさ（Step 5）：太陽の円盤と海面の照り返しのにじみ（ブルーム）とレンズフレア
		Post->Settings.bOverride_BloomIntensity = true;
		Post->Settings.BloomIntensity = 0.8f;
		Post->Settings.bOverride_LensFlareIntensity = true;
		Post->Settings.LensFlareIntensity = 0.5f;

		// 昼と夜（Step 5）：-ShipNight で夜。部品と値を渡しておき、実行時に AShipLighting が切り替える
		AShipLighting* Lighting = World->SpawnActor<AShipLighting>();
		Lighting->SetActorLabel(TEXT("ShipLighting"));
		Lighting->Sun = Sun;
		Lighting->Moon = Moon;
		Lighting->SkyLight = SkyLight;
		Lighting->SkySphere = SkySphere;
		Lighting->Fog = Fog;
		Lighting->Post = Post;
		Lighting->SunDirEnu = SunEnu;
		Lighting->SunLux = SunLux;
		Lighting->SunColor = FLinearColor(SunRgb.X, SunRgb.Y, SunRgb.Z);
		Lighting->SkyNits = SkyNitsPerUnit;
		Lighting->FogColor = FLinearColor(Horizon.X, Horizon.Y, Horizon.Z);
		Lighting->DayEV100 = DaylightEV100;

		// 海面（Step 3）
		AShipOcean* Ocean = World->SpawnActor<AShipOcean>();
		Ocean->SetActorLabel(TEXT("ShipOcean"));
		Ocean->NearMaterial = OceanMat;
		Ocean->FarMaterial = FarMat;

		// 観客の開始位置：原点の南西 約 280 m・高さ 40 m から北東を見る
		APlayerStart* Start = World->SpawnActor<APlayerStart>(ShipFrame::LocalToUnreal(-200.0, -200.0, 40.0),
			ShipFrame::LocalDirToUnreal(1.0, 1.0, -0.15).Rotation());
		Start->SetActorLabel(TEXT("ViewStart"));

		// 船団（Step 2）：実行時に Config/Fleet/fleet_ue.json の船を並べ、-ShipSync のとき箱庭の PDU で動かす
		AShipFleet* Fleet = World->SpawnActor<AShipFleet>();
		Fleet->SetActorLabel(TEXT("ShipFleet"));

		UE_LOG(LogShipEditor, Log, TEXT("scene built: sun %.0f lx (el %.1f deg), sky sphere, sky light, fog, ocean, view start, ship fleet"),
			SunLux, SkyInfo->GetNumberField(TEXT("sun_elevation_deg")));
		return true;
	}

	bool SaveScene(const FString& Map)
	{
		UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
		if (!World)
		{
			return false;
		}
		const bool bMap = UEditorLoadingAndSavingUtils::SaveMap(World, Map);
		const bool bRest = UEditorLoadingAndSavingUtils::SaveDirtyPackages(false, true);
		UE_LOG(LogShipEditor, Log, TEXT("saved map %s (%d) and content (%d)"), *Map, bMap, bRest);
		return bMap && bRest;
	}

	/** 組む → 数十フレーム待つ（シェーダ）→ 保存 →（必要なら）終了。 */
	void RunScheduled(const FString& Map, bool bQuit, int32 StartDelayFrames)
	{
		struct FState { int32 Frame = 0; bool bBuilt = false; };
		TSharedRef<FState> St = MakeShared<FState>();
		FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda([Map, bQuit, StartDelayFrames, St](float) -> bool
		{
			++St->Frame;
			if (!GEditor || St->Frame < StartDelayFrames)
			{
				return true;
			}
			if (!St->bBuilt)
			{
				St->bBuilt = true;
				St->Frame = StartDelayFrames;
				if (!BuildScene())
				{
					UE_LOG(LogShipEditor, Error, TEXT("Ship.BuildScene failed"));
					if (bQuit) { FPlatformMisc::RequestExitWithStatus(false, 1); }
					return false;
				}
				return true;
			}
			if (St->Frame < StartDelayFrames + 30)
			{
				return true;
			}
			const bool bSaved = SaveScene(Map);
			UE_LOG(LogShipEditor, Log, TEXT("Ship.BuildScene %s"), bSaved ? TEXT("OK") : TEXT("save failed"));
			if (bQuit)
			{
				FPlatformMisc::RequestExitWithStatus(false, bSaved ? 0 : 1);
			}
			return false;
		}));
	}
}

class FHakoniwaShipEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		Command = MakeUnique<FAutoConsoleCommand>(
			TEXT("Ship.BuildScene"),
			TEXT("Build the sea scene: [-ShipMap=/Game/...]"),
			FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Argv)
			{
				RunScheduled(ParseMap(*(TEXT(" ") + FString::Join(Argv, TEXT(" ")))), false, 1);
			}));
		const TCHAR* Cmd = FCommandLine::Get();
		if (FParse::Param(Cmd, TEXT("ShipBuildScene")))
		{
			RunScheduled(ParseMap(Cmd), FParse::Param(Cmd, TEXT("ShipQuit")), 120);
		}
	}

	virtual void ShutdownModule() override
	{
		Command.Reset();
	}

private:
	TUniquePtr<FAutoConsoleCommand> Command;
};

IMPLEMENT_MODULE(FHakoniwaShipEditorModule, HakoniwaShipEditor);
