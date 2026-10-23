// ShipModelImport.cpp

#include "ShipModelImport.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "Dom/JsonObject.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "MeshDescription.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "StaticMeshAttributes.h"

DEFINE_LOG_CATEGORY_STATIC(LogShipModel, Log, All);

namespace
{
	const TCHAR* ShipsDir = TEXT("/Game/Ship/Generated/Ships");

	/** glb を JSON と BIN に分ける */
	struct FGlb
	{
		TSharedPtr<FJsonObject> Json;
		TArray<uint8> Bin;

		bool Load(const FString& Path)
		{
			TArray<uint8> B;
			if (!FFileHelper::LoadFileToArray(B, *Path) || B.Num() < 20)
			{
				return false;
			}
			auto U32 = [&B](int32 O) { return static_cast<uint32>(B[O]) | (B[O + 1] << 8) | (B[O + 2] << 16) | (static_cast<uint32>(B[O + 3]) << 24); };
			if (U32(0) != 0x46546C67 || U32(16) != 0x4E4F534A)   // "glTF" / "JSON"
			{
				return false;
			}
			const int32 JsonLen = U32(12);
			const FUTF8ToTCHAR Conv(reinterpret_cast<const ANSICHAR*>(B.GetData() + 20), JsonLen);
			if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(FString(Conv.Length(), Conv.Get())), Json) || !Json)
			{
				return false;
			}
			const int32 BinAt = 20 + JsonLen;
			if (BinAt + 8 <= B.Num() && U32(BinAt + 4) == 0x004E4942)   // "BIN"
			{
				const int32 BinLen = U32(BinAt);
				Bin.Append(B.GetData() + BinAt + 8, BinLen);
			}
			return true;
		}

		/** accessor の要素を double で読む（float / unsigned byte・short・int に対応） */
		bool Read(int32 Accessor, TArray<double>& Out, int32& OutComps) const
		{
			const TSharedPtr<FJsonObject> A = Json->GetArrayField(TEXT("accessors"))[Accessor]->AsObject();
			const TSharedPtr<FJsonObject> V = Json->GetArrayField(TEXT("bufferViews"))[A->GetIntegerField(TEXT("bufferView"))]->AsObject();
			const FString Type = A->GetStringField(TEXT("type"));
			OutComps = Type == TEXT("SCALAR") ? 1 : Type == TEXT("VEC2") ? 2 : Type == TEXT("VEC3") ? 3 : Type == TEXT("VEC4") ? 4 : 0;
			const int32 CT = A->GetIntegerField(TEXT("componentType"));
			const int32 Size = CT == 5126 || CT == 5125 ? 4 : CT == 5123 ? 2 : CT == 5121 ? 1 : 0;
			if (OutComps == 0 || Size == 0)
			{
				return false;
			}
			const int32 Count = A->GetIntegerField(TEXT("count"));
			int32 Stride = OutComps * Size;
			V->TryGetNumberField(TEXT("byteStride"), Stride);
			int32 Base = 0, AOff = 0;
			V->TryGetNumberField(TEXT("byteOffset"), Base);
			A->TryGetNumberField(TEXT("byteOffset"), AOff);
			Base += AOff;
			if (Base + (Count - 1) * Stride + OutComps * Size > Bin.Num())
			{
				return false;
			}
			Out.SetNumUninitialized(Count * OutComps);
			for (int32 i = 0; i < Count; ++i)
			{
				const uint8* P = Bin.GetData() + Base + i * Stride;
				for (int32 c = 0; c < OutComps; ++c)
				{
					const uint8* Q = P + c * Size;
					double Value = 0.0;
					switch (CT)
					{
					case 5126: { float F; FMemory::Memcpy(&F, Q, 4); Value = F; break; }
					case 5125: { uint32 U; FMemory::Memcpy(&U, Q, 4); Value = U; break; }
					case 5123: { uint16 U; FMemory::Memcpy(&U, Q, 2); Value = U; break; }
					default: Value = *Q; break;
					}
					Out[i * OutComps + c] = Value;
				}
			}
			return true;
		}
	};

	/** glTF の点・向き → Unreal（cm）。FLU = (−Z, −X, Y)、Unreal = (100x, −100y, 100z) */
	FVector3f ToUnreal(double X, double Y, double Z, double Scale) { return FVector3f(-Z * Scale, X * Scale, Y * Scale); }

	struct FPartStats
	{
		int32 Triangles = 0;
		int32 Inverted = 0;   // 頂点の法線と面の向き（Unreal の表の向き）が逆の三角形
	};

	/** 1 つのノードのメッシュを MeshDescription にする */
	bool BuildDescription(const FGlb& G, const TSharedPtr<FJsonObject>& Mesh, FMeshDescription& MD, TArray<FName>& OutSlots, FPartStats& Stats)
	{
		FStaticMeshAttributes Attr(MD);
		Attr.Register();
		TVertexAttributesRef<FVector3f> Pos = Attr.GetVertexPositions();
		TVertexInstanceAttributesRef<FVector3f> Nrm = Attr.GetVertexInstanceNormals();
		TVertexInstanceAttributesRef<FVector2f> Uv = Attr.GetVertexInstanceUVs();
		Uv.SetNumChannels(1);
		TPolygonGroupAttributesRef<FName> SlotNames = Attr.GetPolygonGroupMaterialSlotNames();
		const TArray<TSharedPtr<FJsonValue>>& Materials = G.Json->GetArrayField(TEXT("materials"));
		TMap<FName, FPolygonGroupID> Groups;
		for (const TSharedPtr<FJsonValue>& PV : Mesh->GetArrayField(TEXT("primitives")))
		{
			const TSharedPtr<FJsonObject> Prim = PV->AsObject();
			int32 Mode = 4;
			Prim->TryGetNumberField(TEXT("mode"), Mode);
			int32 MatIndex = 0;
			Prim->TryGetNumberField(TEXT("material"), MatIndex);
			const TSharedPtr<FJsonObject> Attrs = Prim->GetObjectField(TEXT("attributes"));
			int32 PosAcc = -1, NrmAcc = -1, IdxAcc = -1;
			Attrs->TryGetNumberField(TEXT("POSITION"), PosAcc);
			Attrs->TryGetNumberField(TEXT("NORMAL"), NrmAcc);
			Prim->TryGetNumberField(TEXT("indices"), IdxAcc);
			if (Mode != 4 || PosAcc < 0 || NrmAcc < 0 || IdxAcc < 0)
			{
				UE_LOG(LogShipModel, Error, TEXT("unsupported primitive (mode %d, POSITION/NORMAL/indices required)"), Mode);
				return false;
			}
			TArray<double> P, N, I;
			int32 PC = 0, NC = 0, IC = 0;
			if (!G.Read(PosAcc, P, PC) || !G.Read(NrmAcc, N, NC) || !G.Read(IdxAcc, I, IC) || PC != 3 || NC != 3 || IC != 1 || P.Num() != N.Num())
			{
				UE_LOG(LogShipModel, Error, TEXT("cannot read accessors"));
				return false;
			}
			const FName Slot(*Materials[MatIndex]->AsObject()->GetStringField(TEXT("name")));
			FPolygonGroupID* Found = Groups.Find(Slot);
			FPolygonGroupID Group;
			if (Found)
			{
				Group = *Found;
			}
			else
			{
				Group = MD.CreatePolygonGroup();
				SlotNames[Group] = Slot;
				Groups.Add(Slot, Group);
				OutSlots.Add(Slot);
			}
			const int32 NV = P.Num() / 3;
			TArray<FVertexInstanceID> Inst;
			Inst.SetNum(NV);
			for (int32 v = 0; v < NV; ++v)
			{
				const FVertexID Vid = MD.CreateVertex();
				Pos[Vid] = ToUnreal(P[3 * v], P[3 * v + 1], P[3 * v + 2], 100.0);
				Inst[v] = MD.CreateVertexInstance(Vid);
				Nrm[Inst[v]] = ToUnreal(N[3 * v], N[3 * v + 1], N[3 * v + 2], 1.0).GetSafeNormal();
				Uv.Set(Inst[v], 0, FVector2f::ZeroVector);
			}
			// glTF（右手系）の反時計回りの表は、鏡映（行列式 −1）の座標の移しで Unreal の表の向きの並びになる（Step 3 の波面メッシュと同じ規約）。
			// 頂点の法線と比べて確かめる（UKismetProceduralMeshLibrary::CalculateTangentsForMesh と同じ面の法線 (P1−P2)×(P0−P2)）
			for (int32 t = 0; t + 2 < I.Num(); t += 3)
			{
				const int32 A = static_cast<int32>(I[t]), B = static_cast<int32>(I[t + 1]), C = static_cast<int32>(I[t + 2]);
				if (A >= NV || B >= NV || C >= NV)
				{
					return false;
				}
				const FVector3f P0 = Pos[MD.GetVertexInstanceVertex(Inst[A])], P1 = Pos[MD.GetVertexInstanceVertex(Inst[B])], P2 = Pos[MD.GetVertexInstanceVertex(Inst[C])];
				const FVector3f Face = FVector3f::CrossProduct(P1 - P2, P0 - P2);
				const FVector3f VN = Nrm[Inst[A]] + Nrm[Inst[B]] + Nrm[Inst[C]];
				if (Face.SizeSquared() > 1e-8f)
				{
					++Stats.Triangles;
					Stats.Inverted += FVector3f::DotProduct(Face, VN) < 0.0f ? 1 : 0;
				}
				MD.CreateTriangle(Group, { Inst[A], Inst[B], Inst[C] });
			}
		}
		return true;
	}

	UStaticMesh* SaveMesh(const FString& Model, const FString& Node, FMeshDescription&& MD, const TArray<FName>& Slots, UMaterialInterface* Paint)
	{
		const FString Name = TEXT("SM_") + Node;
		const FString PackageName = FString(ShipsDir) / Model / Name;
		UStaticMesh* SM = LoadObject<UStaticMesh>(nullptr, *(PackageName + TEXT(".") + Name), nullptr, LOAD_NoWarn | LOAD_Quiet);
		if (!SM)
		{
			UPackage* Package = CreatePackage(*PackageName);
			SM = NewObject<UStaticMesh>(Package, *Name, RF_Public | RF_Standalone | RF_Transactional);
			FAssetRegistryModule::AssetCreated(SM);
		}
		SM->Modify();
		SM->GetStaticMaterials().Reset();
		for (const FName& Slot : Slots)
		{
			SM->GetStaticMaterials().Add(FStaticMaterial(Paint, Slot, Slot));
		}
		if (SM->GetNumSourceModels() == 0)
		{
			SM->AddSourceModel();
		}
		FStaticMeshSourceModel& Src = SM->GetSourceModel(0);
		Src.BuildSettings.bRecomputeNormals = false;   // glb の法線（Godot と同じ見た目）
		Src.BuildSettings.bRecomputeTangents = true;
		Src.BuildSettings.bGenerateLightmapUVs = false;
		Src.BuildSettings.bRemoveDegenerates = false;
		// Nanite：Nanite でないメッシュが多いと仮想シャドウマップの書き込みがあふれる（画面に「[VSM] Non-Nanite Marking Job Queue overflow」）
		FMeshNaniteSettings Nanite = SM->GetNaniteSettings();
		Nanite.bEnabled = true;
		SM->SetNaniteSettings(Nanite);
		SM->CreateMeshDescription(0, MoveTemp(MD));
		SM->CommitMeshDescription(0);
		SM->Build(false);
		SM->PostEditChange();
		SM->MarkPackageDirty();
		return SM;
	}
}

int32 ShipModelImport::BuildShipMeshes(UMaterialInterface* Paint)
{
	const FString FleetPath = FPaths::ProjectDir() / TEXT("Config/Fleet/fleet_ue.json");
	FString Text;
	TSharedPtr<FJsonObject> Fleet;
	if (!FFileHelper::LoadFileToString(Text, *FleetPath) || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Fleet) || !Fleet)
	{
		UE_LOG(LogShipModel, Error, TEXT("not readable: %s"), *FleetPath);
		return -1;
	}
	int32 Built = 0;
	TSet<FString> Done;
	for (const TSharedPtr<FJsonValue>& SV : Fleet->GetArrayField(TEXT("ships")))
	{
		const TSharedPtr<FJsonObject> Ship = SV->AsObject();
		const FString Model = Ship->GetStringField(TEXT("model"));
		if (Done.Contains(Model))
		{
			continue;
		}
		Done.Add(Model);
		TSet<FString> Nodes;
		for (const TSharedPtr<FJsonValue>& P : Ship->GetArrayField(TEXT("parts")))
		{
			Nodes.Add(P->AsObject()->GetStringField(TEXT("node")));
		}
		for (const TSharedPtr<FJsonValue>& J : Ship->GetArrayField(TEXT("joints")))
		{
			Nodes.Add(J->AsObject()->GetStringField(TEXT("node")));
		}
		const FString GlbPath = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() / TEXT("SourceArt/Ships") / (Model + TEXT(".glb")));
		FGlb G;
		if (!G.Load(GlbPath))
		{
			UE_LOG(LogShipModel, Error, TEXT("not a readable glb: %s"), *GlbPath);
			return -1;
		}
		const TArray<TSharedPtr<FJsonValue>>& GNodes = G.Json->GetArrayField(TEXT("nodes"));
		const TArray<TSharedPtr<FJsonValue>>& GMeshes = G.Json->GetArrayField(TEXT("meshes"));
		int32 Tris = 0, Inverted = 0, Parts = 0;
		for (const TSharedPtr<FJsonValue>& NV : GNodes)
		{
			const TSharedPtr<FJsonObject> N = NV->AsObject();
			FString Name;
			int32 MeshIndex = -1;
			if (!N->TryGetStringField(TEXT("name"), Name) || !N->TryGetNumberField(TEXT("mesh"), MeshIndex) || !Nodes.Contains(Name))
			{
				continue;
			}
			FMeshDescription MD;
			TArray<FName> Slots;
			FPartStats Stats;
			if (!BuildDescription(G, GMeshes[MeshIndex]->AsObject(), MD, Slots, Stats))
			{
				UE_LOG(LogShipModel, Error, TEXT("%s/%s: mesh not converted"), *Model, *Name);
				return -1;
			}
			SaveMesh(Model, Name, MoveTemp(MD), Slots, Paint);
			Tris += Stats.Triangles;
			Inverted += Stats.Inverted;
			++Parts;
			++Built;
			Nodes.Remove(Name);
		}
		if (Nodes.Num() > 0)
		{
			UE_LOG(LogShipModel, Error, TEXT("%s: nodes not found in the glb: %s"), *Model, *FString::Join(Nodes.Array(), TEXT(", ")));
			return -1;
		}
		UE_LOG(LogShipModel, Log, TEXT("SHIP_MODEL import %s: %d parts, %d triangles, %d facing against their normals (%.2f %%)"),
			*Model, Parts, Tris, Inverted, Tris > 0 ? 100.0 * Inverted / Tris : 0.0);
	}
	return Built;
}
