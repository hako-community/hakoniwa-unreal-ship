// ShipModelImport.h
// 船の glb（SourceArt/Ships/<model>.glb、tools/gen_fleet_config.py が Godot ビューアから写す）を、部品（メッシュのノード）ごとの
// StaticMesh（/Game/Ship/Generated/Ships/<model>/SM_<node>）にする（SHP-24 Step 4）。
//
// エンジンの glTF 取り込みは使わず、座標を自分で移す：glTF（右手系 Y 上）→ FLU（x = −Z, y = −X, z = Y、gen_fleet_config.py と同じ）
// → Unreal（100x, −100y, 100z、ShipFrame）＝ (−100Z, 100X, 100Y)。頂点はノードの原点からの相対のまま（原点はアクタが部品を置く位置）。
// 材質の差し込み口の名前は glb の材質名。色などは実行時に AShipActor が fleet_ue.json の materials から入れる。

#pragma once

#include "CoreMinimal.h"

class UMaterialInterface;

namespace ShipModelImport
{
	/** fleet_ue.json の船の型ごとに、parts / joints に載っているノードを StaticMesh にする。作った数を返す（失敗は −1） */
	int32 BuildShipMeshes(UMaterialInterface* Paint);
}
