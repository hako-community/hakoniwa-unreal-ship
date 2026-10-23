// HakoniwaShip.Build.cs — 自動運航船の主ビューアの実行時モジュール（SHP-24）。hakoniwa-unreal-rover の HakoniwaRover から始めた。

using UnrealBuildTool;

public class HakoniwaShip : ModuleRules
{
	public HakoniwaShip(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		bEnableExceptions = true;   // hakoniwa-pdu-registry の C++ の変換（std::string・std::vector）

		PublicDependencyModuleNames.AddRange(new string[] {
			"Core", "CoreUObject", "Engine",
		});
		PrivateDependencyModuleNames.AddRange(new string[] { "Json", "RHI", "RenderCore" });
		// 箱庭のアセット（HakoniwaShmClient。hakoniwa-unreal-drone / rover と同じ）：PduManager は公開ヘッダに出る・ゲームパッドを捨てる入力の前処理
		PublicDependencyModuleNames.AddRange(new string[] { "HakoniwaPdu", "InputCore" });
		PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
	}
}
