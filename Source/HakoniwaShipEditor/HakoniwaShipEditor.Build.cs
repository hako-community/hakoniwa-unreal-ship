// HakoniwaShipEditor.Build.cs — 海の場面を組むエディタ用モジュール（SHP-24）。

using UnrealBuildTool;

public class HakoniwaShipEditor : ModuleRules
{
	public HakoniwaShipEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PrivateDependencyModuleNames.AddRange(new string[] {
			"Core", "CoreUObject", "Engine", "UnrealEd", "AssetRegistry",
			"HakoniwaShip",
			// 海面・空の材質と空の画像の取り込み（Step 3）
			"MaterialEditor", "AssetTools", "Json",
		});
	}
}
