using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

namespace PaladinAgent
{
    [DisallowMultipleComponent]
    
    public class PaladinGC : MonoBehaviour
    {
        [HideInInspector] public Paladin agent;

        public bool touchingGround;
        public float groundContactPenalty;
        private const string GroundTag = "ground";
        
        void OnCollisionEnter(Collision col)
        {
            if (col.transform.CompareTag(GroundTag))
            {
                touchingGround = true;

                if (groundContactPenalty == -1)
                {
                    agent.EndEpisode();
                }
            }
        }
        
        void OnCollisionExit(Collision other)
        {
            if (other.transform.CompareTag(GroundTag))
            {
                touchingGround = false;
            }
        }
    }

}