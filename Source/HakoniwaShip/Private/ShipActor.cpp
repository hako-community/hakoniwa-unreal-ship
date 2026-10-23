// ShipActor.cpp

#include "ShipActor.h"

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

void AShipActor::AddPart(USceneComponent* Parent, const TSharedPtr<FJsonObject>& Part, const FVector& OriginFlu, const TSharedPtr<FJsonObject>& Materials)
{
	const FString Node = Part->GetStringField(TEXT("node"));
	const FVector BoxMin = JsonVec3(Part, TEXT("box_min"));
	const FVector BoxMax = JsonVec3(Part, TEXT("box_max"));
	const FString Name = TEXT("SM_") + Node;
	UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *FString::Printf(TEXT("/Game/Ship/Generated/Ships/%s/%s.%s"), *Model, *Name, *Name), nullptr, LOAD_NoWarn | LOAD_Quiet);
	if (!Mesh)
	{
		AddBox(Parent, Node, BoxMin, BoxMax, JsonColor(Part));
		++BoxParts;
		return;
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
	for (const TSharedPtr<FJsonValue>& V : Ship->GetArrayField(TEXT("parts")))
	{
		const TSharedPtr<FJsonObject> P = V->AsObject();
		AddPart(RootComponent, P, JsonVec3(P, TEXT("origin")), Materials);
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
	UE_LOG(LogShipActor, Log, TEXT("SHIP_MODEL %s (%s): %d glb meshes, %d placeholder boxes, %d joints, %d materials, max bounds err %.2e m"),
		*Robot, *Model, MeshParts, BoxParts, Joints.Num(), Paints.Num(), MaxBoundsErr);
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
