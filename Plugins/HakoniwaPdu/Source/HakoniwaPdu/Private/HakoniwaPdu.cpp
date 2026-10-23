// Copyright Epic Games, Inc. All Rights Reserved.

#include "HakoniwaPdu.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformProcess.h"
#include "Logging/LogMacros.h"
#include "Misc/Paths.h"

#ifndef HAKO_CORE_DLL_PATH_FALLBACK
#define HAKO_CORE_DLL_PATH_FALLBACK ""
#endif

#define LOCTEXT_NAMESPACE "FHakoniwaPduModule"
DEFINE_LOG_CATEGORY_STATIC(LogHakoniwaPdu, Log, All);

namespace
{
#if PLATFORM_WINDOWS
FString ResolveShakocDllPath()
{
    TArray<FString> CandidateDirs;

    const FString EnvDir = FPlatformMisc::GetEnvironmentVariable(TEXT("HAKO_CORE_DLL_PATH"));
    if (!EnvDir.IsEmpty())
    {
        CandidateDirs.Add(EnvDir);
    }

    // ★ 2026-10-18（rover）: ビルド時に使った SDK の DLL を、標準配置（%APPDATA%\hakoCore-win）より先に探す。
    //   逆の順だと、環境変数を付けずに開いたとき、ビルドに使っていない古い SDK の DLL を読んでしまう（SDK の混在）。
    const FString BuildDir = FString(UTF8_TO_TCHAR(HAKO_CORE_DLL_PATH_FALLBACK));
    if (!BuildDir.IsEmpty())
    {
        CandidateDirs.Add(BuildDir);
    }

    const FString AppDataDir = FPlatformMisc::GetEnvironmentVariable(TEXT("APPDATA"));
    if (!AppDataDir.IsEmpty())
    {
        CandidateDirs.Add(AppDataDir / TEXT("hakoCore-win/bin"));
    }

    CandidateDirs.Add(FPaths::ProjectDir() / TEXT("Binaries/Win64"));

    for (FString CandidateDir : CandidateDirs)
    {
        FPaths::NormalizeFilename(CandidateDir);
        const FString CandidatePath = FPaths::ConvertRelativePathToFull(CandidateDir / TEXT("shakoc.dll"));
        if (FPaths::FileExists(CandidatePath))
        {
            return CandidatePath;
        }
    }

    return FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() / TEXT("Binaries/Win64/shakoc.dll"));
}
#endif
}

void FHakoniwaPduModule::StartupModule()
{
    UE_LOG(LogHakoniwaPdu, Log, TEXT("HakoniwaPdu: StartupModule called"));
#if PLATFORM_WINDOWS
    const FString DllPath = ResolveShakocDllPath();
    if (FPaths::FileExists(DllPath))
    {
        ShakocDllHandle = FPlatformProcess::GetDllHandle(*DllPath);
        if (ShakocDllHandle == nullptr)
        {
            UE_LOG(LogHakoniwaPdu, Error, TEXT("HakoniwaPdu: failed to load shakoc.dll: %s"), *DllPath);
        }
        else
        {
            UE_LOG(LogHakoniwaPdu, Log, TEXT("HakoniwaPdu: loaded shakoc.dll: %s"), *DllPath);
        }
    }
    else
    {
        UE_LOG(LogHakoniwaPdu, Warning, TEXT("HakoniwaPdu: shakoc.dll not found: %s"), *DllPath);
    }
#endif
}


void FHakoniwaPduModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
#if PLATFORM_WINDOWS
    if (ShakocDllHandle != nullptr)
    {
        FPlatformProcess::FreeDllHandle(ShakocDllHandle);
        ShakocDllHandle = nullptr;
    }
#endif
	UE_LOG(LogHakoniwaPdu, Log, TEXT("HakoniwaPdu: ShutdownModule called"));

}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FHakoniwaPduModule, HakoniwaPdu)
