// ShipActor.h
// 1 隻の船の見た目（SHP-24 Step 2）。姿勢と関節を受け取って描くだけで、箱庭とのやりとりは AShipFleet が受け持つ。
//
// 形は Config/Fleet/fleet_ue.json（tools/gen_fleet_config.py が glb から作る）の parts / joints。
// Step 4：部品は glb から場面づくりが作った StaticMesh（/Game/Ship/Generated/Ships/<model>/SM_<node>）を置き、
// 材質の差し込み口（glb の材質名）に fleet_ue.json の materials の色などを入れる（Godot と同じ見た目）。
// メッシュの外形を fleet_ue.json の外形の箱と比べる（MaxBoundsErr。座標の移しの確かめ）。メッシュが無ければ外形の箱を仮の形として置く。
// 座標：アクタ = 船体の原点（MJCF の vessel・glb のルート）。FLU → Unreal は ShipFrame。

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ShipActor.generated.h"

class FJsonObject;
class UMaterialInterface;

USTRUCT()
struct FShipJoint
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere) FString Name;              // joint_states の関節名
	UPROPERTY(VisibleAnywhere) FString Node;              // glb のノード名
	UPROPERTY(VisibleAnywhere) FVector AxisFlu = FVector::UnitZ();
	UPROPERTY(VisibleAnywhere) TObjectPtr<USceneComponent> Pivot;
	double LastQ = 0.0;
};

UCLASS()
class HAKONIWASHIP_API AShipActor : public AActor
{
	GENERATED_BODY()

public:
	AShipActor();

	/** fleet_ue.json の ships[i] から形を作る */
	bool BuildFromSpec(const TSharedPtr<FJsonObject>& Ship);

	/** pos PDU の値（linear [m]、angular = (roll, asin(R20), yaw) [rad]） */
	void ApplyPose(const FVector& Linear, const FVector& Angular);

	/** joint_states の名前と角度 [rad]。知らない名前は無視し、反映した数を返す */
	int32 ApplyJoints(const TArray<FString>& Names, const TArray<double>& Positions);

	/** 検証用：アクタの変換から PDU の値を読み戻す（ApplyPose の逆。式は ShipFrame の逆変換） */
	void ReadBackPose(FVector& OutLinear, FVector& OutAngular) const;

	/** 検証用：関節の部品の今の角度を読み戻す（軸まわりの回転角）。関節が無ければ false */
	bool ReadBackJoint(const FString& Name, double& OutQ) const;

	UPROPERTY(VisibleAnywhere, Category = "Ship") FString Robot;
	UPROPERTY(VisibleAnywhere, Category = "Ship") FString Model;
	UPROPERTY(VisibleAnywhere, Category = "Ship") TArray<FShipJoint> Joints;

	/** 置いた部品の数（glb のメッシュ / 仮の箱）と、メッシュの外形と fleet_ue.json の外形の箱の差の最大 [m] */
	UPROPERTY(VisibleAnywhere, Category = "Ship") int32 MeshParts = 0;
	UPROPERTY(VisibleAnywhere, Category = "Ship") int32 BoxParts = 0;
	double MaxBoundsErr = 0.0;

private:
	UStaticMeshComponent* AddBox(USceneComponent* Parent, const FString& Name, const FVector& MinFlu, const FVector& MaxFlu, const FLinearColor& Color);
	/** 部品を置く。OriginFlu は親（船体または関節の原点）からの部品の原点（外形の箱は親の座標） */
	void AddPart(USceneComponent* Parent, const TSharedPtr<FJsonObject>& Part, const FVector& OriginFlu, const TSharedPtr<FJsonObject>& Materials);
	UMaterialInterface* Paint(const FString& Slot, UMaterialInterface* Parent, const TSharedPtr<FJsonObject>& Materials);

	UPROPERTY() TObjectPtr<UMaterialInterface> BaseMaterial;
	UPROPERTY() TMap<FName, TObjectPtr<UMaterialInterface>> Paints;
};
