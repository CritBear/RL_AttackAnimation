using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;


namespace PaladinAgent
{
    public class PaladinTargetController : MonoBehaviour
    {
        private string tagToDetect = "weapon";

        [HideInInspector] public Paladin agent;

        private float spawnDegree = 15;
        private float spawnMinDistance = 1.2f;
        private float spawnMaxDistance = 1.8f;
        private float spawnMinHeight = 1.0f;
        private float spawnMaxHeight = 1.8f;

        public float MoveTargetToRandomPosition()
        {
            float angle = Random.Range(-spawnDegree, spawnDegree) * Mathf.Deg2Rad;
            float distance = Random.Range(spawnMinDistance, spawnMaxDistance);
            float height = Random.Range(spawnMinHeight, spawnMaxHeight);

            Vector3 newTargetPos = new Vector3(
                Mathf.Sin(angle) * distance, 
                height, 
                Mathf.Cos(angle) * distance);

            transform.localPosition = newTargetPos;

            return angle;
        }

        void OnTriggerEnter(Collider col)
        {
            if (col.transform.CompareTag(tagToDetect))
            {
                agent.hitTarget = true;
            }
        }
    }
}

