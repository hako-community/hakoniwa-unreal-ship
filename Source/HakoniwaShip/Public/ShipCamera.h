// ShipCamera.h
// 船載カメラ（SHP-25）：自船の取り付け位置（船舶生成器の mounts、glb の mount_<id>）に置いたシーンキャプチャで、画面と同じ見た目
// （ポストプロセス後の色：太陽光のまぶしさ・霧・夜の灯火・露出を含む）の画像を撮り、箱庭の PDU（sensor_msgs/Image、rgb8）へ書き戻す。
//
//   - 撮る時刻：表示時刻（AShipFleet の D24、船と海面がその時刻で描かれている）が 1/rate ごとの区切りを越えたフレーム。
//     画像の header.stamp はそのフレームの表示時刻（物理時刻の軸）。2 台は半周期ずらして撮る（1 フレームに 2 枚撮らない）
//   - 読み出しは非同期（GPU の写しを数フレーム後に受け取る。描画を止めない）。受け取ったら rgb8 に詰めて PDU へ書く
//   - 向きは船首方向・水平（船体に固定。船の揺れと一緒に動く）。画角は水平の画角
//   - -ShipCameraDump=<dir>：書いた画像を 2 s ごとに PNG で保存（目視確認用）

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "ShipCamera.generated.h"

class AActor;
class FJsonObject;
class UPduManager;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

UCLASS()
class HAKONIWASHIP_API UShipCamera : public UObject
{
	GENERATED_BODY()

public:
	/** fleet_ue.json の cameras[i] で作る。Ship は取り付ける船（自船） */
	bool Init(AActor* Ship, const TSharedPtr<FJsonObject>& Spec, double PhaseSec);

	/** 表示時刻 T のフレームで呼ぶ。区切りを越えていれば撮る */
	void MaybeCapture(double T);
	/** 毎フレーム呼ぶ。届いた画像を PDU に書く（Pdu が無ければ数えるだけ） */
	void Poll(UPduManager* Pdu);

	FString Robot;
	FString PduName;
	FString FrameId;
	int32 Width = 640;
	int32 Height = 480;
	double RateHz = 10.0;
	FVector OriginFlu = FVector::ZeroVector;   // 取り付け位置（船体の座標、FLU）
	/** 最後に撮った画像（FOC 画面に映す） */
	UTextureRenderTarget2D* GetTarget() const { return Target; }

	// 状況（AShipFleet が結合テストの終わりに出す）
	int64 Captured = 0;
	int64 Written = 0;
	int64 WriteFailures = 0;
	int64 Dropped = 0;          // 受け取りが追いつかず捨てた
	int32 MaxLatencyFrames = 0; // 撮ってから書くまでのフレーム数の最大
	double LastStamp = -1.0;
	double MeanLuma = 0.0;      // 最後に書いた画像の平均の明るさ（0〜255、真っ黒・真っ白でないことの目安）

private:
	void Write(UPduManager* Pdu, const TArray<FColor>& Bgra, double Stamp);

	UPROPERTY() TObjectPtr<USceneCaptureComponent2D> Capture;
	UPROPERTY() TObjectPtr<UTextureRenderTarget2D> Target;
	TArray<TSharedPtr<struct FShipCameraReadback, ESPMode::ThreadSafe>> Pending;
	double NextCapture = -1.0;
	double Phase = 0.0;
	FString DumpDir;
	double NextDump = 0.0;
	TArray<uint8> Buffer;
};
