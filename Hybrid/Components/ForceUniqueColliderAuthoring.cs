#if UNITY_EDITOR

using System;
using UnityEngine;
using UnityEditor;

namespace Unity.Physics.Authoring
{
    [AddComponentMenu("Entities/Physics/Force Unique Collider")]
    [DisallowMultipleComponent]
    public class ForceUniqueColliderAuthoring : MonoBehaviour
    {
        internal uint ForceUniqueID => m_ForceUniqueID ^ m_PrefabInstanceID;

        [SerializeField, HideInInspector]
        uint m_ForceUniqueID = 0;

        [SerializeField, HideInInspector]
        uint m_PrefabInstanceID = 0;

        const int k_LatestVersion = 2;

        [SerializeField, HideInInspector]
        int m_SerializedVersion = k_LatestVersion;

        void OnEnable()
        {
        }

        void OnValidate()
        {
            if (m_ForceUniqueID == 0)
            {
                m_ForceUniqueID = (uint)UnityEngine.Random.Range(1, Int32.MaxValue);
            }

            var objectId = GlobalObjectId.GetGlobalObjectIdSlow(gameObject);
            var prefabInstanceID = (uint)objectId.targetPrefabId;
            if (prefabInstanceID != m_PrefabInstanceID)
            {
                Undo.RecordObject(this, "Acquire Prefab Instance ID");
                PrefabUtility.RecordPrefabInstancePropertyModifications(this);
                m_PrefabInstanceID = prefabInstanceID;
            }

            if (m_SerializedVersion < k_LatestVersion)
            {
                m_SerializedVersion = k_LatestVersion;
            }
        }
    }
}

#endif
