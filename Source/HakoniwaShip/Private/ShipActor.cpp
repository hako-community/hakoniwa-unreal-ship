// ShipActor.cpp

#include "ShipActor.h"

#include "Components/PointLightComponent.h"
#include "Components/SpotLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "ShipFrame.h"

DEFINE_LOG_CATEGORY_STATIC(LogShipActor, Log, All);

namespace
{
	FVector JsonVec3(const TSharedPtr<FJsonObject>& O, const FString& Key)
	{
		const TArray<TSharedPtr<FJsonValue>>* A = nullptr;
		if (!O->TryGetArrayField(Key, A) || A->Num() != 3)
		{
			return FVector::ZeroVector;
		}
		return FVector((*A)[0]->AsNumber(), (*A)[1]->AsNumber(), (*A)[2]->AsNumber());
	}

	FLinearColor JsonColor(const TSharedPtr<FJsonObject>& O)
	{
		const FVector C = JsonVec3(O, TEXT("color"));
		return FLinearColor::FromSRGBColor(FColor(uint8(C.X * 255.0), uint8(C.Y * 255.0), uint8(C.Z * 255.0)));
	}
}

AShipActor::AShipActor()
{
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Vessel"));
	RootComponent->SetMobility(EComponentMobility::Movable);
}

UStaticMeshComponent* AShipActor::AddBox(USceneComponent* Parent, const FString& Name, const FVector& MinFlu, const FVector& MaxFlu, const FLinearColor& Color)
{
	static UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));   // 1 m の立方体・中心が原点
	if (!BaseMaterial)
	{
		BaseMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	}
	UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(this, *Name);
	C->SetMobility(EComponentMobility::Movable);
	C->SetStaticMesh(Cube);
	C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	C->SetupAttachment(Parent);
	// 箱は軸に沿うので、FLU の min/max を Unreal へ移してから並べ直す（y の符号で入れ替わる）
	const FVector A = ShipFrame::LocalToUnreal(MinFlu);
	const FVector B = ShipFrame::LocalToUnreal(MaxFlu);
	const FVector Lo = A.ComponentMin(B), Hi = A.ComponentMax(B);
	C->SetRelativeLocation((Lo + Hi) * 0.5);
	C->SetRelativeScale3D(((Hi - Lo) / 100.0).ComponentMax(FVector(0.01)));
	if (UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(BaseMaterial, this))
	{
		Mid->SetVectorParameterValue(TEXT("Color"), Color);
		C->SetMaterial(0, Mid);
	}
	C->RegisterComponent();
	AddInstanceComponent(C);
	return C;
}

UMaterialInterface* AShipActor::Paint(const FString& Slot, UMaterialInterface* Parent, const TSharedPtr<FJsonObject>& Materials)
{
	// glb の材質ごとに 1 つ（同じ船の部品で共有）。値は fleet_ue.json の materials（glTF の色は線形）
	const FName Key(*Slot);
	if (TObjectPtr<UMaterialInterface>* Found = Paints.Find(Key))
	{
		return *Found;
	}
	const TSharedPtr<FJsonObject>* M = nullptr;
	UMaterialInterface* Result = Parent;
	if (Materials && Materials->TryGetObjectField(Slot, M))
	{
		if (UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Parent, this))
		{
			const FVector C = JsonVec3(*M, TEXT("base_color"));
			const FVector E = JsonVec3(*M, TEXT("emissive"));
			Mid->SetVectorParameterValue(TEXT("BaseColor"), FLinearColor(C.X, C.Y, C.Z));
			Mid->SetScalarParameterValue(TEXT("Metallic"), (*M)->GetNumberField(TEXT("metallic")));
			Mid->SetScalarParameterValue(TEXT("Roughness"), (*M)->GetNumberField(TEXT("roughness")));
			Mid->SetVectorParameterValue(TEXT("Emissive"), FLinearColor(E.X, E.Y, E.Z));
			Result = Mid;
		}
	}
	else
	{
		UE_LOG(LogShipActor, Warning, TEXT("%s: material %s not in fleet_ue.json"), *Robot, *Slot);
	}
	Paints.Add(Key, Result);
	return Result;
}

UStaticMeshComponent* AShipActor::AddPart(USceneComponent* Parent, const TSharedPtr<FJsonObject>& Part, const FVector& OriginFlu, const TSharedPtr<FJsonObject>& Materials)
{
	const FString Node = Part->GetStringField(TEXT("node"));
	const FVector BoxMin = JsonVec3(Part, TEXT("box_min"));
	const FVector BoxMax = JsonVec3(Part, TEXT("box_max"));
	const FString Name = TEXT("SM_") + Node;
	UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *FString::Printf(TEXT("/Game/Ship/Generated/Ships/%s/%s.%s"), *Model, *Name, *Name), nullptr, LOAD_NoWarn | LOAD_Quiet);
	if (!Mesh)
	{
		++BoxParts;
		return AddBox(Parent, Node, BoxMin, BoxMax, JsonColor(Part));
	}
	UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(this, *Node);
	C->SetMobility(EComponentMobility::Movable);
	C->SetStaticMesh(Mesh);
	C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	C->SetupAttachment(Parent);
	C->SetRelativeLocation(ShipFrame::LocalToUnreal(OriginFlu));
	const TArray<FStaticMaterial>& Slots = Mesh->GetStaticMaterials();
	for (int32 i = 0; i < Slots.Num(); ++i)
	{
		C->SetMaterial(i, Paint(Slots[i].MaterialSlotName.ToString(), Slots[i].MaterialInterface, Materials));
	}
	C->RegisterComponent();
	AddInstanceComponent(C);
	++MeshParts;
	// 座標の移しの確かめ：メッシュの外形（Unreal の cm）を FLU に戻して部品の原点を足したものが、glb から求めた外形の箱と一致するか
	const FBox B = Mesh->GetBoundingBox();
	const FVector A = ShipFrame::UnrealToLocal(B.Min), Z = ShipFrame::UnrealToLocal(B.Max);
	const FVector Lo = A.ComponentMin(Z) + OriginFlu, Hi = A.ComponentMax(Z) + OriginFlu;
	const double Err = FMath::Max((Lo - BoxMin).GetAbsMax(), (Hi - BoxMax).GetAbsMax());
	MaxBoundsErr = FMath::Max(MaxBoundsErr, Err);
	if (Err > 1.0e-3)
	{
		UE_LOG(LogShipActor, Warning, TEXT("%s/%s: mesh bounds differ from fleet_ue.json by %.4f m"), *Robot, *Node, Err);
	}
	return C;
}

void AShipActor::AddNavLight(UStaticMeshComponent* Lamp, const TSharedPtr<FJsonObject>& Spec, const TSharedPtr<FJsonObject>& Materials)
{
	FShipNavLight L;
	L.Node = Spec->GetStringField(TEXT("node"));
	L.Color = Spec->GetStringField(TEXT("color"));
	L.CenterDeg = Spec->GetNumberField(TEXT("center_deg"));
	L.ArcDeg = Spec->GetNumberField(TEXT("arc_deg"));
	L.IntensityCd = Spec->GetNumberField(TEXT("intensity_cd"));
	L.Lamp = Lamp;
	// 灯の輝度 [cd/m²] = 光度 / 横から見た灯の面積（幅 × 高さ）。glb の発光の値（emissiveFactor × strength）の大きさで割って EmissiveNits にする
	const FBox B = Lamp->GetStaticMesh() ? Lamp->GetStaticMesh()->GetBoundingBox() : FBox(FVector(-12.0), FVector(12.0));
	const FVector Size = B.GetSize() / 100.0;
	const double Area = FMath::Max(1.0e-4, FMath::Max(Size.X, Size.Y) * Size.Z);
	for (int32 i = 0; i < Lamp->GetNumMaterials(); ++i)
	{
		// 灯ごとに材質を分ける（マスト灯と船尾灯は同じ材質）。★ 動的インスタンスは動的インスタンスを親にできないので、元の材質から作って値を入れ直す
		UMaterialInterface* Base = Lamp->GetMaterial(i);
		UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Base ? Base->GetMaterial() : nullptr, this);
		if (!Mid)
		{
			continue;
		}
		const FString Slot = Lamp->GetStaticMesh() ? Lamp->GetStaticMesh()->GetStaticMaterials()[i].MaterialSlotName.ToString() : FString();
		const TSharedPtr<FJsonObject>* M = nullptr;
		FVector E = FVector::ZeroVector;
		if (Materials && Materials->TryGetObjectField(Slot, M))
		{
			const FVector C = JsonVec3(*M, TEXT("base_color"));
			E = JsonVec3(*M, TEXT("emissive"));
			Mid->SetVectorParameterValue(TEXT("BaseColor"), FLinearColor(C.X, C.Y, C.Z));
			Mid->SetScalarParameterValue(TEXT("Metallic"), (*M)->GetNumberField(TEXT("metallic")));
			Mid->SetScalarParameterValue(TEXT("Roughness"), (*M)->GetNumberField(TEXT("roughness")));
			Mid->SetVectorParameterValue(TEXT("Emissive"), FLinearColor(E.X, E.Y, E.Z));
		}
		const double EMax = E.GetMax();
		Mid->SetScalarParameterValue(TEXT("EmissiveNits"), EMax > 0.0 ? L.IntensityCd / Area / EMax : 0.0);
		Mid->SetScalarParameterValue(TEXT("SectorCenter"), L.CenterDeg);
		Mid->SetScalarParameterValue(TEXT("SectorHalf"), L.ArcDeg * 0.5);
		Lamp->SetMaterial(i, Mid);
	}
	// 光源：COLREG の色（白は白熱灯の暖色）、光度 [cd]。射光範囲 160° 以下はスポット（中心の向き・半角 = 範囲の半分）
	const FLinearColor Color = L.Color == TEXT("red") ? FLinearColor(1.0f, 0.06f, 0.03f)
		: L.Color == TEXT("green") ? FLinearColor(0.05f, 1.0f, 0.45f) : FLinearColor(1.0f, 0.9f, 0.75f);
	ULocalLightComponent* Light = nullptr;
	if (L.ArcDeg <= 160.0)
	{
		USpotLightComponent* Spot = NewObject<USpotLightComponent>(this, *(L.Node + TEXT("_light")));
		Spot->SetOuterConeAngle(L.ArcDeg * 0.5);
		Spot->SetInnerConeAngle(FMath::Max(0.0, L.ArcDeg * 0.5 - 2.0));
		Spot->SetRelativeRotation(FRotator(0.0, L.CenterDeg, 0.0));   // Unreal の yaw は右回り（+Y = 右舷）
		Light = Spot;
	}
	else
	{
		Light = NewObject<UPointLightComponent>(this, *(L.Node + TEXT("_light")));
	}
	Light->SetMobility(EComponentMobility::Movable);
	Light->SetupAttachment(Lamp);
	Light->SetIntensityUnits(ELightUnits::Candelas);
	Light->SetIntensity(L.IntensityCd);
	Light->SetLightColor(Color);
	Light->SetAttenuationRadius(6000.0f);
	Light->SetCastShadows(false);
	Light->SetVisibility(false);
	Light->RegisterComponent();
	AddInstanceComponent(Light);
	L.Light = Light;
	NavLights.Add(L);
}

void AShipActor::SetNight(bool bNight)
{
	for (FShipNavLight& L : NavLights)
	{
		if (L.Light)
		{
			L.Light->SetVisibility(bNight);
		}
	}
}

bool AShipActor::BuildFromSpec(const TSharedPtr<FJsonObject>& Ship)
{
	Robot = Ship->GetStringField(TEXT("robot"));
	Model = Ship->GetStringField(TEXT("model"));
#if WITH_EDITOR
	SetActorLabel(Robot);
#endif
	const TSharedPtr<FJsonObject>* MaterialsPtr = nullptr;
	const TSharedPtr<FJsonObject> Materials = Ship->TryGetObjectField(TEXT("materials"), MaterialsPtr) ? *MaterialsPtr : nullptr;
	TMap<FString, TSharedPtr<FJsonObject>> NavSpecs;
	const TArray<TSharedPtr<FJsonValue>>* NavArray = nullptr;
	if (Ship->TryGetArrayField(TEXT("nav_lights"), NavArray))
	{
		for (const TSharedPtr<FJsonValue>& V : *NavArray)
		{
			NavSpecs.Add(V->AsObject()->GetStringField(TEXT("node")), V->AsObject());
		}
	}
	for (const TSharedPtr<FJsonValue>& V : Ship->GetArrayField(TEXT("parts")))
	{
		const TSharedPtr<FJsonObject> P = V->AsObject();
		UStaticMeshComponent* C = AddPart(RootComponent, P, JsonVec3(P, TEXT("origin")), Materials);
		if (const TSharedPtr<FJsonObject>* Nav = NavSpecs.Find(P->GetStringField(TEXT("node"))); Nav && C)
		{
			AddNavLight(C, *Nav, Materials);
		}
	}
	for (const TSharedPtr<FJsonValue>& V : Ship->GetArrayField(TEXT("joints")))
	{
		const TSharedPtr<FJsonObject> J = V->AsObject();
		FShipJoint Joint;
		Joint.Name = J->GetStringField(TEXT("name"));
		Joint.Node = J->GetStringField(TEXT("node"));
		Joint.AxisFlu = JsonVec3(J, TEXT("axis"));
		USceneComponent* Pivot = NewObject<USceneComponent>(this, *(Joint.Node + TEXT("_pivot")));
		Pivot->SetMobility(EComponentMobility::Movable);
		Pivot->SetupAttachment(RootComponent);
		Pivot->SetRelativeLocation(ShipFrame::LocalToUnreal(JsonVec3(J, TEXT("origin"))));
		Pivot->RegisterComponent();
		AddInstanceComponent(Pivot);
		const TSharedPtr<FJsonObject> P = J->GetObjectField(TEXT("part"));
		AddPart(Pivot, P, FVector::ZeroVector, Materials);   // 関節の部品の外形の箱は関節の原点からの相対
		Joint.Pivot = Pivot;
		Joints.Add(Joint);
	}
	UE_LOG(LogShipActor, Log, TEXT("SHIP_MODEL %s (%s): %d glb meshes, %d placeholder boxes, %d joints, %d materials, %d nav lights, max bounds err %.2e m"),
		*Robot, *Model, MeshParts, BoxParts, Joints.Num(), Paints.Num(), NavLights.Num(), MaxBoundsErr);
	return true;
}

void AShipActor::ApplyPose(const FVector& Linear, const FVector& Angular)
{
	const FMatrix R = ShipFrame::PduAngularToRotation(Angular.X, Angular.Y, Angular.Z);
	// ★ UE は回転の変化が 1e-4°（FRotator::Equals の既定の許容）未満だと回転を書き換えない（SceneComponent.cpp）。
	//   表示時刻で毎フレーム補間すると 1 フレームの変化がそれより小さいことがあり、最大 約 2e-6 rad 遅れる（見えない。検証の許容 1e-4 の内）
	SetActorLocationAndRotation(ShipFrame::LocalToUnreal(Linear), ShipFrame::RotationToUnreal(R));
}

int32 AShipActor::ApplyJoints(const TArray<FString>& Names, const TArray<double>& Positions)
{
	int32 Applied = 0;
	for (FShipJoint& J : Joints)
	{
		const int32 i = Names.IndexOfByKey(J.Name);
		if (i == INDEX_NONE || i >= Positions.Num() || !J.Pivot)
		{
			continue;
		}
		J.LastQ = Positions[i];
		J.Pivot->SetRelativeRotation(ShipFrame::RotationToUnreal(ShipFrame::RotAxis(J.AxisFlu, J.LastQ)));
		++Applied;
	}
	return Applied;
}

void AShipActor::ReadBackPose(FVector& OutLinear, FVector& OutAngular) const
{
	OutLinear = ShipFrame::UnrealToLocal(GetActorLocation());
	OutAngular = ShipFrame::RotationToPduAngular(ShipFrame::UnrealToRotation(GetActorQuat()));
}

bool AShipActor::ReadBackJoint(const FString& Name, double& OutQ) const
{
	for (const FShipJoint& J : Joints)
	{
		if (J.Name != Name || !J.Pivot)
		{
			continue;
		}
		// 軸 n まわりの回転 R：sin q = n·vee(R − Rᵀ)/2、cos q = (tr R − 1)/2
		const FMatrix R = ShipFrame::UnrealToRotation(J.Pivot->GetRelativeRotation().Quaternion());
		const FVector Vee(R.M[2][1] - R.M[1][2], R.M[0][2] - R.M[2][0], R.M[1][0] - R.M[0][1]);
		const double S = FVector::DotProduct(J.AxisFlu.GetSafeNormal(), Vee) * 0.5;
		const double C = (R.M[0][0] + R.M[1][1] + R.M[2][2] - 1.0) * 0.5;
		OutQ = FMath::Atan2(S, C);
		return true;
	}
	return false;
}
