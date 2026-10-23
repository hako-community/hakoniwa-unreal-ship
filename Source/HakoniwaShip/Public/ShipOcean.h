// ShipOcean.h
// 物理と同じ決定論的波面を描く海面（SHP-24 Step 3、D4・D19）。Godot ビューアの Scripts/ocean.gd と同じ考え方:
//   - 波浪パラメータは SetWaveParams()（Disturbance PDU、無ければ fleet_ue.json の wave.preview）。物理プラグインと同じ規則
//     （Hs > active_hs_m で有効、num_components 成分）で wave_spectrum.hpp（物理と同じコード）が展開する
//   - 時刻は SetPhysicsTime()（OwnShip の joint_states の stamp、D19）。呼ばれないとき（オフラインの確認）はゲームの時刻
//   - メッシュ（2.4 km 四方・4 m 格子）はカメラに付いて動くが、格子の間隔で位置をそろえるので頂点はいつも同じ世界の点を標本にする。
//     外側は 40 km 四方の平面（潮位面、内側は描かない。傾き（法線）は同じ波の式で出す）。★ 180 km にすると平面に光が当たらなくなった（黒い帯）
//   - 見た目だけの細かい波（Ripple）は法線にだけ足す（高さは物理の波面のまま）
//
// 検証（-ShipSurfaceCheck、または AShipFleet の結合テスト）：真上からの正射影の深度で、描いた海面の高さを読み、
// CPU（倍精度）の η(x, y, t) と頂点ごとに比べる（Godot の wave_shader_parity に当たるものを、実際に描いた海面で行う）。
// 読み出しは非同期（GPU の写しを数フレーム後に受け取る）。同期で読むと描画が止まり、10 s ごとに 28 ms のカクつきが出た。
// -ShipWaveParity=<reference.json>：wave_spectrum.hpp の結果を Godot の tests/unit/wave_parity/reference.json（Python 実装の値）と比べる。

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ShipWave.h"
#include "ShipOcean.generated.h"

class UMaterialInstanceDynamic;
class UMaterialInterface;
class UProceduralMeshComponent;
class USceneCaptureComponent2D;
class UStaticMeshComponent;
class UTexture2D;
class UTextureRenderTarget2D;

UCLASS()
class HAKONIWASHIP_API AShipOcean : public AActor
{
	GENERATED_BODY()

public:
	AShipOcean();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void Tick(float DeltaSeconds) override;

	UPROPERTY(EditAnywhere, Category = "Ocean") TObjectPtr<UMaterialInterface> NearMaterial;
	UPROPERTY(EditAnywhere, Category = "Ocean") TObjectPtr<UMaterialInterface> FarMaterial;
	UPROPERTY(EditAnywhere, Category = "Ocean") double MeshSizeM = 2400.0;
	UPROPERTY(EditAnywhere, Category = "Ocean") int32 Cells = 600;
	UPROPERTY(EditAnywhere, Category = "Ocean") double FarSizeM = 40000.0;
	/** 外周で振幅を 0 へ落とす（Godot の fade_start / fade_end / hole_radius と同じ）。検証は FadeStartM の内側 */
	UPROPERTY(EditAnywhere, Category = "Ocean") double FadeStartM = 950.0;
	UPROPERTY(EditAnywhere, Category = "Ocean") double FadeEndM = 1180.0;
	UPROPERTY(EditAnywhere, Category = "Ocean") double HoleRadiusM = 1150.0;
	/** 1 で Gerstner の水平変位（波頭が尖るが物理の標高とは一致しなくなる。見た目用）。既定 0 */
	UPROPERTY(EditAnywhere, Category = "Ocean") float HorizontalDisplacement = 0.0f;
	/** 細かい波（見た目だけ）の数と強さ（傾きの倍率） */
	UPROPERTY(EditAnywhere, Category = "Ocean") int32 RippleCount = 16;
	UPROPERTY(EditAnywhere, Category = "Ocean") float RippleStrength = 1.0f;

	/** 物理プラグインと同じ規則の成分数・有効化の閾値（fleet_ue.json の wave） */
	void Configure(int32 NumComponents, double ActiveHs);
	/** 波浪パラメータを展開する（同じ値なら何もしない） */
	void SetWaveParams(const ShipWave::FParams& P);
	/** 海面の時刻を物理時刻にする（D19）。以後ゲームの時刻は使わない */
	void SetPhysicsTime(double T);

	bool IsWaveActive() const { return bActive; }
	const ShipWave::FParams& GetWaveParams() const { return Params; }
	int32 ComponentCount() const { return bActive ? static_cast<int32>(Engine.get_components().size()) : 0; }
	double GetTime() const { return TimeSec; }
	/** 物理と同じ CPU 倍精度の標高 [m]（ENU）。波浪が無効なら潮位 */
	double ElevationAt(double East, double North, double T) const;

	/** 次の Tick の始めに、描いた海面の高さを真上から読んで CPU の値と比べる */
	void RequestSurfaceCheck() { bSurfaceCheckRequested = true; }
	int32 SurfaceChecks = 0;
	int64 SurfacePoints = 0;
	int64 SurfaceMisses = 0;      // 海面が写らなかった画素
	double SurfaceMaxErr = 0.0;   // [m]
	double SurfaceMaxEta = 0.0;   // 比べた点の |η| の最大 [m]（波が描かれていることの確認）

private:
	void BuildMeshes();
	void CreateWaveTexture();
	void UpdateGpu();
	void StartSurfaceCheck();
	void PollSurfaceCheck();
	void FinishSurfaceCheck(const TArray<FLinearColor>& Px, const FVector& Cam, const FVector2D& Center, double Time);
	static bool RunParity(const FString& Path);

	UPROPERTY() TObjectPtr<UProceduralMeshComponent> Near;
	UPROPERTY() TObjectPtr<UStaticMeshComponent> Far;
	UPROPERTY() TObjectPtr<UTexture2D> WaveTex;
	UPROPERTY() TObjectPtr<UMaterialInstanceDynamic> NearMid;
	UPROPERTY() TObjectPtr<UMaterialInstanceDynamic> FarMid;
	UPROPERTY() TObjectPtr<USceneCaptureComponent2D> Capture;
	UPROPERTY() TObjectPtr<UTextureRenderTarget2D> CaptureTarget;

	ShipWave::FEngine Engine;
	ShipWave::FParams Params;
	bool bHasParams = false;
	bool bActive = false;
	int32 NumComponents = ShipWave::DefaultComponents;
	double ActiveHs = ShipWave::DefaultActiveHs;

	double TimeSec = 0.0;
	bool bPhysicsTime = false;
	double StepM = 4.0;
	FVector2D CenterEnu = FVector2D::ZeroVector;   // メッシュの中心（東, 北）[m]

	struct FRipple { double Kx, Ky, Amp, Omega, Phase; };
	TArray<FRipple> Ripples;

	// 描画に渡した状態（検証は次のフレームの始めに、この状態と比べる）
	bool bRendered = false;
	double RenderedTime = 0.0;
	FVector2D RenderedCenter = FVector2D::ZeroVector;

	bool bSurfaceCheckRequested = false;
	TSharedPtr<struct FShipSurfaceReadback, ESPMode::ThreadSafe> PendingCheck;   // 受け取り待ちの検証
	bool bAutoSurfaceCheck = false;
	double NextAutoCheck = 2.0;
};
