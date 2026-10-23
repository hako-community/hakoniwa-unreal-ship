// Copyright Epic Games, Inc. All Rights Reserved.

using System;
using System.IO;
using UnrealBuildTool;

public class HakoniwaPdu : ModuleRules
{
    private static string EscapeDefinitionValue(string Value)
    {
        return Value.Replace("\\", "\\\\").Replace("\"", "\\\"");
    }

    private static string GetEnvPath(string Name)
    {
        string Value = Environment.GetEnvironmentVariable(Name);
        return string.IsNullOrWhiteSpace(Value) ? null : Value;
    }

    private static string ResolveDirectory(string EnvName, string DefaultPath, string RequiredFile, string FallbackPath)
    {
        string EnvPath = GetEnvPath(EnvName);
        if (EnvPath != null && File.Exists(Path.Combine(EnvPath, RequiredFile)))
        {
            return EnvPath;
        }

        if (!string.IsNullOrWhiteSpace(DefaultPath) && File.Exists(Path.Combine(DefaultPath, RequiredFile)))
        {
            return DefaultPath;
        }

        return FallbackPath;
    }

    private static string ResolveFile(string EnvName, string DefaultPath, string FileName, string FallbackPath)
    {
        string Directory = ResolveDirectory(EnvName, DefaultPath, FileName, null);
        return Directory != null ? Path.Combine(Directory, FileName) : FallbackPath;
    }

	public HakoniwaPdu(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		bEnableExceptions = true;
		string DefaultCoreRoot = Path.Combine(
			Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
			"hakoCore-win");
		string CoreIncludePath = ResolveDirectory(
			"HAKO_CORE_INC_PATH",
			Path.Combine(DefaultCoreRoot, "include"),
			"hako_capi.h",
			Path.Combine(ModuleDirectory, "../ThirdParty/shakoc/include"));
		
		PublicIncludePaths.AddRange(
			new string[] {
                Path.Combine(ModuleDirectory, "../ThirdParty/hakoniwa-pdu-registry/pdu/types"),
                CoreIncludePath
				// ... add public include paths required here ...
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core", "WebSockets", "Json", "JsonUtilities"
				// ... add other public dependencies that you statically link with here ...
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
				// ... add private dependencies that you statically link with here ...	
			}
			);
        if (Target.bBuildEditor)
        {
            PrivateDependencyModuleNames.Add("UnrealEd");
        }
        // ★ 2026-10-03（A1）: Android では hakoniwa-pdu-registry のヘッダが Unreal の型と衝突する・C の版の判定で
        //   未定義の識別子の警告（エラー扱い）になる。サブモジュールは変えずに、ここで避ける（HakoPduTypesCompat.h）。
        if (Target.Platform == UnrealTargetPlatform.Android)
        {
            ForceIncludeFiles.Add(Path.Combine(Path.Combine(ModuleDirectory, "Public"), "HakoPduTypesCompat.h"));
            CppCompileWarningSettings.UndefinedIdentifierWarningLevel = WarningLevel.Off;
        }


        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            string LibPath = ResolveFile(
                "HAKO_CORE_LIB_PATH",
                Path.Combine(DefaultCoreRoot, "lib"),
                "shakoc.lib",
                Path.Combine(ModuleDirectory, "../ThirdParty/shakoc/lib/Win64/shakoc.lib"));
            PublicAdditionalLibraries.Add(LibPath);

            string DllPath = ResolveFile(
                "HAKO_CORE_DLL_PATH",
                Path.Combine(DefaultCoreRoot, "bin"),
                "shakoc.dll",
                Path.Combine(ModuleDirectory, "../../../../Binaries/Win64/shakoc.dll"));
            if (File.Exists(DllPath))
            {
                RuntimeDependencies.Add(DllPath);
            }
            PublicDefinitions.Add("HAKO_SHM_ENABLE=1");
            PublicDefinitions.Add("HAKO_CORE_DLL_PATH_FALLBACK=\"" + EscapeDefinitionValue(Path.GetDirectoryName(DllPath)) + "\"");
        }

        DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
