#if TOOLS
using Godot;
using Godot.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;

namespace Mmd.addons.MMDImport
{
    [Tool]
    public partial class PMXImporter : EditorSceneFormatImporter
    {
        class CreateModelContext
        {
            public bool UseBlendShape;
            public bool UseDynamicLight;
            public string basePath;
            public HashSet<int> blendShapeVertex;
            public System.Collections.Generic.Dictionary<PMX_Material, Material> materialMap;
            public Godot.Collections.Array<Godot.Collections.Dictionary> appendBoneMeta;
            public Godot.Collections.Array<Godot.Collections.Dictionary> ikBoneMeta;
            public Godot.Collections.Array<Godot.Collections.Dictionary> physicsBoneMeta;
            public Godot.Collections.Array<Godot.Collections.Dictionary> jointMeta = new Array<Dictionary>();
            public Godot.Collections.Array<Godot.Collections.Dictionary> materialMeta = new Array<Dictionary>();
            public Godot.Collections.Dictionary morphMeta = new Dictionary();
            public Godot.Collections.Dictionary options;

        }

        public override string[] _GetExtensions()
        {
            return new string[] { "pmx" };
        }
        public override Variant _GetOptionVisibility(string path, bool forAnimation, string option)
        {
            return base._GetOptionVisibility(path, forAnimation, option);
        }
        public override uint _GetImportFlags()
        {
            return (uint)EditorSceneFormatImporter.ImportScene;
        }

        public override GodotObject _ImportScene(string path, uint flags, Dictionary options)
        {
            float modelScale = 0.1f;

            var basePath = path[..(path.LastIndexOf('/'))];
            var bytes = Godot.FileAccess.GetFileAsBytes(path);
            using var stream = new MemoryStream(bytes);
            PMXFormat pmx = PMXFormat.Load(new System.IO.BinaryReader(stream));
            pmx.Scale(modelScale);

            PackedScene p = ResourceLoader.Load<PackedScene>("res://addons/MMDImport/mmd_model.tscn");
            var root = p.Instantiate<Node3D>();
            root.Name = pmx.Name;
            root.SetMeta("description", pmx.Description);
            root.SetMeta("description_en", pmx.DescriptionEN);
            root.SetMeta("model_scale", modelScale);

            var createModelContext = new CreateModelContext()
            {
                basePath = basePath,
                options = options,
            };
            createModelContext.UseDynamicLight = CheckCharacter(pmx);
            CollectMorphVertex(pmx, createModelContext);
            CollectMaterial(pmx, createModelContext);
            var skeleton = root.GetChild<Skeleton3D>(0);
            CreateBones(skeleton, pmx, createModelContext);
            var skin = new Skin();
            CreateSkin(skin, pmx);

            createModelContext.UseBlendShape = true;
            var m1 = skeleton.GetChild<MeshInstance3D>(0);
            m1.Skin = skin;
            CreateMeshMorph(m1, pmx, createModelContext);

            createModelContext.UseBlendShape = false;
            CreateMesh(skeleton, skin, pmx, createModelContext);

            root.SetMeta("append_bone", createModelContext.appendBoneMeta);
            root.SetMeta("ik_bone", createModelContext.ikBoneMeta);
            root.SetMeta("physics_bone", createModelContext.physicsBoneMeta);
            root.SetMeta("joint", createModelContext.jointMeta);
            root.SetMeta("material", createModelContext.materialMeta);
            root.SetMeta("morph", createModelContext.morphMeta);

            return root;
        }

        void CollectMorphVertex(PMXFormat pmx, CreateModelContext createModelContext)
        {
            HashSet<int> morphVertice = new HashSet<int>();
            createModelContext.blendShapeVertex = morphVertice;

            foreach (var morph in pmx.Morphs)
            {
                var mv = morph.MorphVertice;
                if (mv == null)
                    continue;
                foreach (var d in mv)
                {
                    morphVertice.Add(d.VertexIndex);
                }
            }

            foreach (var morph in pmx.Morphs)
            {
                if (morph.MorphBones != null)
                {
                    var arr = new Godot.Collections.Array();
                    foreach (var b in morph.MorphBones)
                    {
                        arr.Add(new Godot.Collections.Dictionary()
                        {
                            {"type","bone" },
                            {"bone",b.BoneIndex },
                            {"translation",GetVector3(b.Translation) },
                            {"rotation",GetQuaternion(b.Rotation) }
                        });
                    }
                    createModelContext.morphMeta.Add(morph.Name, arr);
                }
                if (morph.SubMorphs != null)
                {
                    var arr = new Godot.Collections.Array();
                    foreach (var s in morph.SubMorphs)
                    {
                        arr.Add(new Godot.Collections.Dictionary()
                        {
                            {"type","group" },
                            {"morph",pmx.Morphs[s.GroupIndex].Name },
                            {"rate",s.Rate }
                        });
                    }
                    createModelContext.morphMeta.Add(morph.Name, arr);
                }
            }
            GD.Print($"Import PMX. morph vertices: {morphVertice.Count}");
        }

        void CollectMaterial(PMXFormat pmx, CreateModelContext createModelContext)
        {
            createModelContext.materialMap = new System.Collections.Generic.Dictionary<PMX_Material, Material>();

            for (int i = 0; i < pmx.Materials.Count; i++)
            {
                PMX_Material material = pmx.Materials[i];
                createModelContext.materialMeta.Add(new Dictionary()
                {
                    {"flags",(int)material.DrawFlags }
                });
            }
            Godot.Collections.Dictionary materials = null;
            if (createModelContext.options.TryGetValue("_subresources", out var _subResource))
            {
                Godot.Collections.Dictionary subresources = _subResource.AsGodotDictionary();
                if (subresources.TryGetValue("materials", out var mat1))
                {
                    materials = mat1.AsGodotDictionary();
                }
            }
            for (int i = 0; i < pmx.Materials.Count; i++)
            {
                PMX_Material material = pmx.Materials[i];
                if (materials != null && materials.TryGetValue(material.Name, out var _mat))
                {
                    var mat = _mat.AsGodotDictionary();
                    if (mat["use_external/enabled"].AsBool())
                    {
                        createModelContext.materialMap[material] = ResourceLoader.Load<Material>(mat["use_external/path"].AsString());
                        continue;
                    }
                }

                var m2 = new StandardMaterial3D();
                m2.TextureFilter = BaseMaterial3D.TextureFilterEnum.LinearWithMipmapsAnisotropic;
                m2.CullMode = material.DrawFlags.HasFlag(PMX_DrawFlag.DrawDoubleFace) ? BaseMaterial3D.CullModeEnum.Disabled : BaseMaterial3D.CullModeEnum.Back;

                bool transparent = false;
                int i3 = material.TextureIndex;
                if (i3 >= 0 && i3 < pmx.Textures.Count)
                {
                    try
                    {
                        var image = Image.LoadFromFile(createModelContext.basePath + '/' + pmx.Textures[i3].TexturePath);
                        transparent = DetectTransparent(image, pmx, material);
                        var imageTexture = GD.Load<Texture2D>(createModelContext.basePath + '/' + pmx.Textures[i3].TexturePath);
                        m2.AlbedoTexture = imageTexture;
                    }
                    catch
                    {

                    }
                }
                if (transparent)
                {
                    m2.Transparency = BaseMaterial3D.TransparencyEnum.Alpha;
                }
                m2.DepthDrawMode = BaseMaterial3D.DepthDrawModeEnum.Always;
                m2.ResourceName = material.Name;
                m2.RenderPriority = i;
                createModelContext.materialMap[material] = m2;
            }
        }

        //bool DetectTransparent(Image image)
        //{
        //    int t1 = 0;
        //    int t2 = 0;
        //    int stride = Mathf.Max(image.GetWidth() / 10, 1);
        //    for (int i = 0; i < image.GetWidth(); i += stride)
        //    {
        //        for (int j = 0; j < image.GetHeight(); j += stride)
        //        {
        //            var color = image.GetPixel(i, j);
        //            if (color.A < 0.9f)
        //            {
        //                t1++;
        //            }
        //            t2++;
        //        }
        //    }
        //    return (float)t1 / (float)t2 > 0.05f;
        //}

        bool DetectTransparent(Image image, PMXFormat pmx, PMX_Material material)
        {
            int t1 = 0;
            int t2 = 1;
            int stride = Mathf.Max(image.GetWidth() / 10, 1);

            int width = image.GetWidth();
            int height = image.GetHeight();
            for (int i = 0; i < material.TriangeIndexNum; i++)
            {
                int i1 = i + material.TriangeIndexStartNum;
                var v = pmx.Vertices[pmx.TriangleIndexs[i1]];
                var a = Mathf.Clamp(Mathf.RoundToInt((v.UvCoordinate.X - Mathf.Floor(v.UvCoordinate.X)) * width), 0, width - 1);
                var b = Mathf.Clamp(Mathf.RoundToInt((v.UvCoordinate.Y - Mathf.Floor(v.UvCoordinate.Y)) * height), 0, height - 1);
                var color = image.GetPixel(a, b);
                if (color.A < 0.9f)
                {
                    t1++;
                }
                t2++;
            }
            return (float)t1 / (float)t2 > 0.05f;
        }

        void CreateMeshMorph(MeshInstance3D meshInstance3D, PMXFormat pmx, CreateModelContext createModelContext)
        {
            var mesh = new ArrayMesh();
            meshInstance3D.Mesh = mesh;
            meshInstance3D.GIMode = createModelContext.UseDynamicLight ? GeometryInstance3D.GIModeEnum.Dynamic : GeometryInstance3D.GIModeEnum.Static;
            if (createModelContext.UseBlendShape)
            {
                foreach (var morph in pmx.Morphs)
                {
                    if (morph.MorphVertice == null)
                        continue;
                    mesh.AddBlendShape(morph.Name);
                }
                mesh.BlendShapeMode = Mesh.BlendShapeMode.Relative;
            }
            for (int i = 0; i < pmx.Materials.Count; i++)
            {
                if (AddSurface(mesh, pmx, pmx.Materials[i], createModelContext))
                {
                    int a = mesh.GetSurfaceCount() - 1;
                    meshInstance3D.SetSurfaceOverrideMaterial(a, mesh.SurfaceGetMaterial(a));
                }
            }
            mesh.ResourceName = pmx.Name;
        }

        void CreateMesh(Skeleton3D skeleton, Skin skin, PMXFormat pmx, CreateModelContext createModelContext)
        {
            for (int i = 0; i < pmx.Materials.Count; i++)
            {
                var meshInstance3D = new MeshInstance3D();
                meshInstance3D.Skin = skin;
                meshInstance3D.GIMode = createModelContext.UseDynamicLight ? GeometryInstance3D.GIModeEnum.Dynamic : GeometryInstance3D.GIModeEnum.Static;

                var mesh = new ArrayMesh();
                meshInstance3D.Mesh = mesh;
                if (AddSurface(mesh, pmx, pmx.Materials[i], createModelContext))
                {
                    int a = mesh.GetSurfaceCount() - 1;
                    meshInstance3D.SetSurfaceOverrideMaterial(a, mesh.SurfaceGetMaterial(a));
                    meshInstance3D.Name = pmx.Materials[i].Name;
                    mesh.ResourceName = pmx.Materials[i].Name;
                    AddChildO(skeleton, meshInstance3D);
                }
                if (!createModelContext.UseDynamicLight)
                    mesh.LightmapUnwrap(Transform3D.Identity, 0.2f);
            }
        }

        bool AddSurface(ArrayMesh mesh, PMXFormat pmx, PMX_Material material, CreateModelContext createModelContext)
        {
            int indexStart = material.TriangeIndexStartNum;

            List<int> modifiedIndex = new List<int>();

            for (int i = 0; i < material.TriangeIndexNum; i += 3)
            {
                int i1 = i + indexStart;
                bool anyBlend = false;
                anyBlend |= createModelContext.blendShapeVertex.Contains(pmx.TriangleIndexs[i1]);
                anyBlend |= createModelContext.blendShapeVertex.Contains(pmx.TriangleIndexs[i1 + 1]);
                anyBlend |= createModelContext.blendShapeVertex.Contains(pmx.TriangleIndexs[i1 + 2]);
                if (createModelContext.UseBlendShape == anyBlend)
                {
                    modifiedIndex.Add(pmx.TriangleIndexs[i1]);
                    modifiedIndex.Add(pmx.TriangleIndexs[i1 + 1]);
                    modifiedIndex.Add(pmx.TriangleIndexs[i1 + 2]);
                }
            }
            if (modifiedIndex.Count == 0)
                return false;

            var c1 = new HashSet<int>();

            for (int i = 0; i < modifiedIndex.Count; i++)
            {
                c1.Add(modifiedIndex[i]);
            }
            var indexMap = c1.ToList();
            var remap = new System.Collections.Generic.Dictionary<int, int>();
            for (int i = 0; i < indexMap.Count; i++)
            {
                int i1 = indexMap[i];
                remap[i1] = i;
            }
            int[] index = new int[modifiedIndex.Count];
            for (int i = 0; i < modifiedIndex.Count; i++)
            {
                int si = modifiedIndex[i];
                index[i] = remap[si];
            }

            Vector3[] positions = new Vector3[indexMap.Count];
            Vector3[] normals = new Vector3[indexMap.Count];
            Vector2[] uvs = new Vector2[indexMap.Count];
            int[] bones = new int[indexMap.Count * 4];
            float[] weights = new float[indexMap.Count * 4];
            //float[] tangents = new float[indexMap.Count * 4];

            for (int i = 0; i < indexMap.Count; i++)
            {
                int i2 = indexMap[i];

                ref var vertex = ref pmx.Vertices[i2];

                positions[i] = GetVector3(vertex.Coordinate);
                normals[i] = GetVector3(vertex.Normal);
                uvs[i] = GetVector2(vertex.UvCoordinate);
                bones[i * 4 + 0] = vertex.boneId0;
                bones[i * 4 + 1] = vertex.boneId1;
                bones[i * 4 + 2] = vertex.boneId2;
                bones[i * 4 + 3] = vertex.boneId3;
                weights[i * 4 + 0] = vertex.Weights.X;
                weights[i * 4 + 1] = vertex.Weights.Y;
                weights[i * 4 + 2] = vertex.Weights.Z;
                weights[i * 4 + 3] = vertex.Weights.W;
                //tangents[i * 4 + 0] = 1;
                //tangents[i * 4 + 1] = 0;
                //tangents[i * 4 + 2] = 0;
                //tangents[i * 4 + 3] = 1;
            }
            var tangents = ComputeTangent(positions, normals, uvs, index);
            for (int i = 0; i < bones.Length; i++)
            {
                if (bones[i] >= pmx.Bones.Count || bones[i] < 0)
                {
                    bones[i] = 0;
                }
            }
            var surfaceArray = new Godot.Collections.Array();
            surfaceArray.Resize((int)ArrayMesh.ArrayType.Max);
            surfaceArray[0] = positions;
            surfaceArray[1] = normals;
            surfaceArray[2] = tangents;
            surfaceArray[4] = uvs;
            surfaceArray[10] = bones;
            surfaceArray[11] = weights;
            surfaceArray[12] = index;

            var blendShapes = new Godot.Collections.Array<Godot.Collections.Array>();
            blendShapes.Resize(mesh.GetBlendShapeCount());

            if (createModelContext.UseBlendShape)
            {
                int a1 = 0;
                foreach (var morph in pmx.Morphs)
                {
                    if (morph.MorphVertice == null)
                        continue;
                    Vector3[] morphPosition = new Vector3[indexMap.Count];
                    foreach (var morphVertex in morph.MorphVertice)
                    {
                        if (remap.TryGetValue(morphVertex.VertexIndex, out int i1))
                            morphPosition[i1] = GetVector3(morphVertex.Offset);
                    }
                    var arr1 = new Godot.Collections.Array();
                    arr1.Resize(3);
                    arr1[0] = morphPosition;
                    arr1[1] = normals;
                    arr1[2] = tangents;
                    //arr1[4] = uvs;

                    blendShapes[a1] = arr1;
                    a1++;
                }
            }

            mesh.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, surfaceArray, blendShapes);
            int surfaceIndex = mesh.GetSurfaceCount() - 1;
            mesh.SurfaceSetName(surfaceIndex, material.Name);


            mesh.SurfaceSetMaterial(surfaceIndex, createModelContext.materialMap[material]);

            return true;
        }


        //bool AddSurface(ArrayMesh mesh, PMXFormat pmx, PMX_Material material, CreateModelContext createModelContext)
        //{
        //    int indexStart = material.TriangeIndexStartNum;

        //    List<int> modifiedIndex = new List<int>();

        //    for (int i = 0; i < material.TriangeIndexNum; i += 3)
        //    {
        //        int i1 = i + indexStart;
        //        bool anyBlend = false;
        //        anyBlend |= createModelContext.blendShapeVertex.Contains(pmx.TriangleIndexs[i1]);
        //        anyBlend |= createModelContext.blendShapeVertex.Contains(pmx.TriangleIndexs[i1 + 1]);
        //        anyBlend |= createModelContext.blendShapeVertex.Contains(pmx.TriangleIndexs[i1 + 2]);
        //        if (createModelContext.UseBlendShape == anyBlend)
        //        {
        //            modifiedIndex.Add(pmx.TriangleIndexs[i1]);
        //            modifiedIndex.Add(pmx.TriangleIndexs[i1 + 1]);
        //            modifiedIndex.Add(pmx.TriangleIndexs[i1 + 2]);
        //        }
        //    }
        //    if (modifiedIndex.Count == 0)
        //        return false;

        //    var c1 = new HashSet<int>();

        //    for (int i = 0; i < modifiedIndex.Count; i++)
        //    {
        //        c1.Add(modifiedIndex[i]);
        //    }
        //    var indexMap = c1.ToList();
        //    var remap = new System.Collections.Generic.Dictionary<int, int>();
        //    for (int i = 0; i < indexMap.Count; i++)
        //    {
        //        int i1 = indexMap[i];
        //        remap[i1] = i;
        //    }
        //    int[] index = new int[modifiedIndex.Count];
        //    for (int i = 0; i < modifiedIndex.Count; i++)
        //    {
        //        int si = modifiedIndex[i];
        //        index[i] = remap[si];
        //    }

        //    SurfaceTool surfaceTool = new SurfaceTool();

        //    //Vector3[] positions = new Vector3[indexMap.Count];
        //    //Vector3[] normals = new Vector3[indexMap.Count];
        //    //Vector2[] uvs = new Vector2[indexMap.Count];
        //    //int[] bones = new int[indexMap.Count * 4];
        //    //float[] weights = new float[indexMap.Count * 4];
        //    surfaceTool.Begin(Mesh.PrimitiveType.Triangles);
        //    for (int i = 0; i < indexMap.Count; i++)
        //    {
        //        int i2 = indexMap[i];

        //        ref var vertex = ref pmx.Vertices[i2];

        //        //positions[i] = GetVector3(vertex.Coordinate);
        //        //normals[i] = GetVector3(vertex.Normal);
        //        //uvs[i] = GetVector2(vertex.UvCoordinate);
        //        //bones[i * 4 + 0] = vertex.boneId0;
        //        //bones[i * 4 + 1] = vertex.boneId1;
        //        //bones[i * 4 + 2] = vertex.boneId2;
        //        //bones[i * 4 + 3] = vertex.boneId3;
        //        //weights[i * 4 + 0] = vertex.Weights.X;
        //        //weights[i * 4 + 1] = vertex.Weights.Y;
        //        //weights[i * 4 + 2] = vertex.Weights.Z;
        //        //weights[i * 4 + 3] = vertex.Weights.W;

        //        surfaceTool.SetUV(GetVector2(vertex.UvCoordinate));
        //        surfaceTool.SetNormal(GetVector3(vertex.Normal));
        //        surfaceTool.SetBones(new int[] { CheckN1(vertex.boneId0), CheckN1(vertex.boneId1), CheckN1(vertex.boneId2), CheckN1(vertex.boneId3) });
        //        surfaceTool.SetWeights(new float[] { vertex.Weights.X, vertex.Weights.Y, vertex.Weights.Z, vertex.Weights.W });
        //        surfaceTool.AddVertex(GetVector3(vertex.Coordinate));
        //    }
        //    foreach (var i in index)
        //        surfaceTool.AddIndex(i);
        //    surfaceTool.GenerateTangents();
        //    var surfaceArray = surfaceTool.CommitToArrays();
        //    //var tangents = ComputeTangent(positions, normals, uvs, index);
        //    //for (int i = 0; i < bones.Length; i++)
        //    //{
        //    //    if (bones[i] >= pmx.Bones.Count || bones[i] < 0)
        //    //    {
        //    //        bones[i] = 0;
        //    //    }
        //    //}
        //    //var surfaceArray = new Godot.Collections.Array();
        //    //surfaceArray.Resize((int)ArrayMesh.ArrayType.Max);
        //    //surfaceArray[0] = positions;
        //    //surfaceArray[1] = normals;
        //    //surfaceArray[2] = tangents;
        //    //surfaceArray[4] = uvs;
        //    //surfaceArray[10] = bones;
        //    //surfaceArray[11] = weights;
        //    //surfaceArray[12] = index;


        //    var blendShapes = new Godot.Collections.Array<Godot.Collections.Array>();
        //    blendShapes.Resize(mesh.GetBlendShapeCount());

        //    if (createModelContext.UseBlendShape)
        //    {
        //        var normals = surfaceArray[1];
        //        var tangents = surfaceArray[2];
        //        int a1 = 0;
        //        foreach (var morph in pmx.Morphs)
        //        {
        //            if (morph.MorphVertice == null)
        //                continue;
        //            Vector3[] morphPosition = new Vector3[indexMap.Count];
        //            foreach (var morphVertex in morph.MorphVertice)
        //            {
        //                if (remap.TryGetValue(morphVertex.VertexIndex, out int i1))
        //                    morphPosition[i1] = GetVector3(morphVertex.Offset);
        //            }
        //            var arr1 = new Godot.Collections.Array();
        //            arr1.Resize(3);
        //            arr1[0] = morphPosition;
        //            arr1[1] = normals;
        //            arr1[2] = tangents;
        //            //arr1[4] = uvs;

        //            blendShapes[a1] = arr1;
        //            a1++;
        //        }
        //    }

        //    mesh.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, surfaceArray, blendShapes);
        //    int surfaceIndex = mesh.GetSurfaceCount() - 1;
        //    mesh.SurfaceSetName(surfaceIndex, material.Name);


        //    mesh.SurfaceSetMaterial(surfaceIndex, createModelContext.materialMap[material]);

        //    return true;
        //}

        void CreateBones(Skeleton3D skeleton, PMXFormat pmx, CreateModelContext createModelContext)
        {
            var appendBoneMeta = new Array<Dictionary>();
            var ikBoneMeta = new Array<Dictionary>();
            var boneAttachments = new List<BoneAttachment3D>();
            for (int i = 0; i < pmx.Bones.Count; i++)
            {
                PMX_Bone bone = pmx.Bones[i];
                int index = skeleton.GetBoneCount();
                skeleton.AddBone(bone.Name);

                if (bone.ParentIndex >= 0 && bone.ParentIndex < pmx.Bones.Count)
                {
                    var parent = pmx.Bones[bone.ParentIndex];
                    skeleton.SetBoneParent(index, bone.ParentIndex);
                    skeleton.SetBoneRest(index, new Transform3D(Basis.Identity, GetVector3(bone.Position - parent.Position)));
                }
                else
                {
                    skeleton.SetBoneRest(index, new Transform3D(Basis.Identity, GetVector3(bone.Position)));
                }

                if (bone.boneIK != null)
                {
                    var array = new Godot.Collections.Array();
                    foreach (var link in bone.boneIK.IKLinks)
                    {
                        var dict1 = new Godot.Collections.Dictionary()
                        {
                            {"index", link.LinkedIndex},
                            {"min",GetVector3(link.LimitMin) },
                            {"max",GetVector3(link.LimitMax) },
                            {"limited",link.HasLimit },
                        };
                        array.Add(dict1);
                    }

                    var dict = new Godot.Collections.Dictionary()
                    {
                        {"bone",i },
                        {"target",bone.boneIK.IKTargetIndex },
                        {"iterate_limit",bone.boneIK.CCDIterateLimit },
                        {"angle_limit",bone.boneIK.CCDAngleLimit },
                        {"links",array }
                    };
                    ikBoneMeta.Add(dict);

                    var tipBone = pmx.Bones[bone.boneIK.IKTargetIndex];

                    //var skeletonIK = new SkeletonIK3D();
                    //skeletonIK.RootBone = pmx.Bones[bone.boneIK.IKLinks[^1].LinkedIndex].Name;
                    //skeletonIK.TipBone = tipBone.Name;
                    //skeletonIK.OverrideTipBasis = false;
                    //skeletonIK.Name = bone.Name;
                    //skeletonIK.TargetNode = $"../IK_{bone.Name}";
                    //skeletonIK.SetMeta("bone_name", bone.Name);
                    //AddChildO(skeleton, skeletonIK);
                    //if (bone.Name == "左足ＩＫ" || bone.Name == "右足ＩＫ")
                    //{
                    //    skeletonIK.UseMagnet = true;
                    //    skeletonIK.Magnet = new Vector3(bone.Position.X, 20, -10);
                    //}
                    //else
                    //{
                    //}
                    //var ikTarget = new BoneAttachment3D();
                    //ikTarget.Name = $"IK_{bone.Name}";
                    //ikTarget.BoneName = bone.Name;
                    //AddChildO(skeleton, ikTarget);
                }

                if (bone.AppendBoneIndex >= 0 && bone.AppendBoneIndex < pmx.Bones.Count)
                {
                    bool ar = bone.Flags.HasFlag(PMX_BoneFlag.AcquireRotate);
                    bool at = bone.Flags.HasFlag(PMX_BoneFlag.AcquireTranslate);
                    if (ar || at)
                    {
                        appendBoneMeta.Add(new Dictionary()
                        {
                            {"bone",i },
                            {"ar",ar },
                            {"at",at },
                            {"index",bone.AppendBoneIndex },
                            {"ratio",bone.AppendBoneRatio },
                        });
                    }
                }
            }
            createModelContext.appendBoneMeta = appendBoneMeta;
            createModelContext.ikBoneMeta = ikBoneMeta;

            //var physicsBinded = new System.Collections.Generic.Dictionary<PMX_Bone, PhysicsBody3D>();
            //var physicsBindedName = new System.Collections.Generic.Dictionary<int, string>();


            var physicsBoneMeta = new Array<Dictionary>();
            createModelContext.physicsBoneMeta = physicsBoneMeta;
            for (int i = 0; i < pmx.RigidBodies.Count; i++)
            {
                PMX_RigidBody rigidBody = pmx.RigidBodies[i];
                Godot.Collections.Dictionary pMeta = new Godot.Collections.Dictionary();
                pMeta["name"] = rigidBody.Name;
                pMeta["bone_index"] = rigidBody.AssociatedBoneIndex;
                pMeta["collision_group"] = rigidBody.CollisionGroup;
                pMeta["collision_mask"] = rigidBody.CollisionMask;
                pMeta["position"] = GetVector3(rigidBody.Position);
                pMeta["rotation"] = GetVector3(rigidBody.Rotation);
                pMeta["dimensions"] = GetVector3(rigidBody.Dimensions);
                pMeta["linear_damp"] = rigidBody.LinearDamp;
                pMeta["angular_damp"] = rigidBody.AngularDamp;
                pMeta["friction"] = rigidBody.Friction;
                pMeta["bounce"] = rigidBody.Bounce;
                pMeta["mass"] = rigidBody.Mass;
                pMeta["type"] = (int)rigidBody.Type;
                pMeta["shape"] = (int)rigidBody.Shape;
                physicsBoneMeta.Add(pMeta);


                //var b1 = pmx.Bones[rigidBody.AssociatedBoneIndex];
                //if (physicsBinded.TryGetValue(b1, out var pb))
                //{
                //}
                //else
                //{
                //    var physicsBone = new PhysicalBone3D();
                //    physicsBone.Name = $"pb_{rigidBody.Name}";
                //    physicsBone.Mass = rigidBody.Mass;
                //    physicsBone.LinearDamp = rigidBody.LinearDamp;
                //    physicsBone.AngularDamp = rigidBody.AngularDamp;
                //    physicsBone.Bounce = rigidBody.Bounce;
                //    physicsBone.Friction = rigidBody.Friction;
                //    physicsBone.Set("bone_name", b1.Name);
                //    if (rigidBody.Type == PMX_RigidBodyType.Kinematic)
                //    {
                //        physicsBone.SetMeta("is_kinematic", true);
                //    }
                //    physicsBone.CollisionMask = rigidBody.CollisionMask;
                //    physicsBone.CollisionLayer = rigidBody.CollisionGroup;
                //    AddChildO(skeleton, physicsBone);
                //    pb = physicsBone;
                //    physicsBinded[b1] = physicsBone;
                //}
                //physicsBindedName[i] = pb.Name;
                //CollisionShape3D collision = new CollisionShape3D();
                //collision.Position = GetVector3(rigidBody.Position - b1.Position);
                //collision.Rotation = GetVector3(rigidBody.Rotation);
                //AddChildO(pb, collision);
                //switch (rigidBody.Shape)
                //{
                //    case PMX_RigidBodyShape.Sphere:
                //        collision.Shape = new SphereShape3D()
                //        {
                //            Radius = rigidBody.Dimensions.X
                //        };
                //        collision.Name = "Sphere";
                //        break;
                //    case PMX_RigidBodyShape.Box:
                //        collision.Shape = new BoxShape3D()
                //        {
                //            Size = GetVector3(rigidBody.Dimensions)
                //        };
                //        collision.Name = "Box";
                //        break;
                //    case PMX_RigidBodyShape.Capsule:
                //        collision.Shape = new CapsuleShape3D()
                //        {
                //            Radius = rigidBody.Dimensions.X,
                //            Height = rigidBody.Dimensions.Y
                //        };
                //        collision.Name = "Capsule";
                //        break;
                //}

                //MeshInstance3D collisionVisualizer = new MeshInstance3D();
                //collisionVisualizer.Position = collision.Position;
                //collisionVisualizer.Quaternion = collision.Quaternion;

                //switch (rigidBody.Shape)
                //{
                //    case PMX_RigidBodyShape.Sphere:
                //        var sphere = new SphereMesh();
                //        sphere.Radius = rigidBody.Dimemsions.X;
                //        sphere.Height = rigidBody.Dimemsions.X;
                //        collisionVisualizer.Mesh = sphere;
                //        break;
                //    case PMX_RigidBodyShape.Box:
                //        var box = new BoxMesh();
                //        box.Size = GetVector3(rigidBody.Dimemsions);
                //        collisionVisualizer.Mesh = box;
                //        break;
                //    case PMX_RigidBodyShape.Capsule:
                //        var capsule = new CapsuleMesh();
                //        capsule.Radius = rigidBody.Dimemsions.X;
                //        capsule.Height = rigidBody.Dimemsions.Y;
                //        collisionVisualizer.Mesh = capsule;
                //        break;
                //}
                //AddChildO(pb, collisionVisualizer);
            }


            for (int i = 0; i < pmx.Joints.Count; i++)
            {
                PMX_Joint joint = pmx.Joints[i];
                if (joint.AssociatedRigidBodyIndex1 < 0 || joint.AssociatedRigidBodyIndex1 >= pmx.RigidBodies.Count ||
                    joint.AssociatedRigidBodyIndex2 < 0 || joint.AssociatedRigidBodyIndex2 >= pmx.RigidBodies.Count)
                {
                    GD.Print($"invalid joint: {joint.Name}");
                    continue;
                }
                Godot.Collections.Dictionary pMeta = new Godot.Collections.Dictionary();
                pMeta["name"] = joint.Name;
                pMeta["bone1"] = joint.AssociatedRigidBodyIndex1;
                pMeta["bone2"] = joint.AssociatedRigidBodyIndex2;
                pMeta["position"] = GetVector3(joint.Position);
                pMeta["rotation"] = GetVector3(joint.Rotation);
                pMeta["linear_min"] = GetVector3(joint.LinearMinimum);
                pMeta["linear_max"] = GetVector3(joint.LinearMaximum);
                pMeta["linear_spring"] = GetVector3(joint.LinearSpring);
                pMeta["angular_min"] = GetVector3(joint.AngularMinimum);
                pMeta["angular_max"] = GetVector3(joint.AngularMaximum);
                pMeta["angular_spring"] = GetVector3(joint.AngularSpring);
                pMeta["type"] = (byte)joint.Type;
                createModelContext.jointMeta.Add(pMeta);

                //CreateJoint(joint, skeleton, physicsBindedName);
            }
        }

        bool CheckCharacter(PMXFormat pmx)
        {
            HashSet<string> bonesName = new HashSet<string>()
            {
                "右手首",
                "首",
                "右足"
            };
            for (int i = 0; i < pmx.Bones.Count; i++)
            {
                if (bonesName.Contains(pmx.Bones[i].Name))
                    return true;
            }
            return false;
        }

        //void CreateJoint(PMX_Joint joint, Skeleton3D skeleton, System.Collections.Generic.Dictionary<int, string> physicsBindedName)
        //{
        //    string GetPath(int index)
        //    {
        //        return $"../{physicsBindedName[index]}";
        //    }
        //    var j1 = new Generic6DofJoint3D();
        //    j1.Name = joint.Name;
        //    j1.Position = GetVector3(joint.Position);
        //    j1.Rotation = GetVector3(joint.Rotation);
        //    j1.NodeA = GetPath(joint.AssociatedRigidBodyIndex1);
        //    j1.NodeB = GetPath(joint.AssociatedRigidBodyIndex2);
        //    j1.Set("linear_limit_x/upper_distance", joint.LinearMaximum.X);
        //    j1.Set("linear_limit_y/upper_distance", joint.LinearMaximum.Y);
        //    j1.Set("linear_limit_z/upper_distance", joint.LinearMaximum.Z);
        //    j1.Set("linear_limit_x/lower_distance", joint.LinearMinimum.X);
        //    j1.Set("linear_limit_y/lower_distance", joint.LinearMinimum.Y);
        //    j1.Set("linear_limit_z/lower_distance", joint.LinearMinimum.Z);

        //    j1.Set("angular_limit_x/upper_angle", joint.AngularMaximum.X);
        //    j1.Set("angular_limit_y/upper_angle", joint.AngularMaximum.Y);
        //    j1.Set("angular_limit_z/upper_angle", joint.AngularMaximum.Z);
        //    j1.Set("angular_limit_x/lower_angle", joint.AngularMinimum.X);
        //    j1.Set("angular_limit_y/lower_angle", joint.AngularMinimum.Y);
        //    j1.Set("angular_limit_z/lower_angle", joint.AngularMinimum.Z);

        //    if (joint.LinearSpring.X != 0)
        //        j1.Set("linear_spring_x/enabled", true);
        //    if (joint.LinearSpring.Y != 0)
        //        j1.Set("linear_spring_y/enabled", true);
        //    if (joint.LinearSpring.Z != 0)
        //        j1.Set("linear_spring_z/enabled", true);
        //    j1.Set("linear_spring_x/stiffness", joint.LinearSpring.X);
        //    j1.Set("linear_spring_y/stiffness", joint.LinearSpring.Y);
        //    j1.Set("linear_spring_z/stiffness", joint.LinearSpring.Z);
        //    j1.Set("linear_limit_x/softness", 0.9f);
        //    j1.Set("linear_limit_y/softness", 0.9f);
        //    j1.Set("linear_limit_z/softness", 0.9f);

        //    if (joint.AngularSpring.X != 0)
        //        j1.Set("angular_spring_x/enabled", true);
        //    if (joint.AngularSpring.Y != 0)
        //        j1.Set("angular_spring_y/enabled", true);
        //    if (joint.AngularSpring.Z != 0)
        //        j1.Set("angular_spring_z/enabled", true);
        //    j1.Set("angular_spring_x/stiffness", joint.AngularSpring.X);
        //    j1.Set("angular_spring_y/stiffness", joint.AngularSpring.Y);
        //    j1.Set("angular_spring_z/stiffness", joint.AngularSpring.Z);
        //    AddChildO(skeleton, j1);
        //}

        void CreateSkin(Skin skin, PMXFormat pmx)
        {
            for (int i = 0; i < pmx.Bones.Count; i++)
            {
                var bone = pmx.Bones[i];
                skin.AddNamedBind(bone.Name, new Transform3D(Basis.Identity, GetVector3(-bone.Position)));
            }
        }

        static Quaternion GetQuaternion(System.Numerics.Quaternion quat)
        {
            return new Quaternion(quat.X, quat.Y, quat.Z, quat.W);
        }

        static Vector3 GetVector3(System.Numerics.Vector3 vec3)
        {
            return new Vector3(vec3.X, vec3.Y, vec3.Z);
        }

        static Vector2 GetVector2(System.Numerics.Vector2 vec2)
        {
            return new Vector2(vec2.X, vec2.Y);
        }

        static void AddChildO(Node parent, Node child)
        {
            parent.AddChild(child);
            child.Owner = parent.Owner;
        }


        float[] ComputeTangent(Vector3[] position, Vector3[] normal, Vector2[] uv, int[] index)
        {
            int vertexCount = position.Length;

            Vector3[] bitangent = new Vector3[vertexCount];
            Vector4[] tangent = new Vector4[vertexCount];

            for (int i = 0; i < vertexCount; i++)
            {
                tangent[i] = new Vector4(0.0F, 0.0F, 0.0F, 0.0F);
            }
            for (int i = 0; i < vertexCount; i++)
            {
                bitangent[i] = new Vector3(0.0F, 0.0F, 0.0F);
            }

            // Calculate tangent and bitangent for each triangle and add to all three vertices.
            for (int k = 0; k < index.Length; k += 3)
            {
                int i0 = index[k];
                int i1 = index[k + 1];
                int i2 = index[k + 2];
                Vector3 p0 = position[i0];
                Vector3 p1 = position[i1];
                Vector3 p2 = position[i2];
                Vector2 w0 = uv[i0];
                Vector2 w1 = uv[i1];
                Vector2 w2 = uv[i2];
                Vector3 e1 = p1 - p0;
                Vector3 e2 = p2 - p0;
                float x1 = w1.X - w0.X, x2 = w2.X - w0.X;
                float y1 = w1.Y - w0.Y, y2 = w2.Y - w0.Y;
                float r = 1.0F / (x1 * y2 - x2 * y1);
                Vector3 t = (e1 * y2 - e2 * y1) * r;
                Vector3 b = (e2 * x1 - e1 * x2) * r;
                Vector4 t1 = new Vector4(t.X, t.Y, t.Z, 0);
                tangent[i0] += t1;
                tangent[i1] += t1;
                tangent[i2] += t1;
                bitangent[i0] += b;
                bitangent[i1] += b;
                bitangent[i2] += b;
            }

            for (int i = 0; i < vertexCount; i++)
            {
                float factor;
                Vector3 t1 = bitangent[i].Cross(normal[i]);
                if (t1.Dot(new Vector3(tangent[i].X, tangent[i].Y, tangent[i].Z)) > 0)
                    factor = 1;
                else
                    factor = -1;
                t1 = t1.Normalized() * factor;
                tangent[i] = new Vector4(t1.X, t1.Y, t1.Z, 1);
            }
            return MemoryMarshal.Cast<Vector4, float>(new System.Span<Vector4>(tangent)).ToArray();
        }

        int CheckN1(int i)
        {
            return Mathf.Max(i, 0);
        }
    }
}
#endif