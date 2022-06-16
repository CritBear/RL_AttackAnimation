using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using Unity.MLAgents;

namespace Unity.MLAgentsExamples
{
    [System.Serializable]
    public class BodyPart
    {
        [Header("Body Part Info")] [Space(10)] public ArticulationBody joint;
        // public Rigidbody rb;
        [HideInInspector] public ArticulationReducedSpace startingPos;
        [HideInInspector] public Quaternion startingRot;

        [Header("Ground & Target Contact")]
        [Space(10)]
        public GroundContact groundContact;

        public TargetContact targetContact;

        [FormerlySerializedAs("thisJDController")]
        [HideInInspector] public JointDriveController thisJdController;

        [Header("Current Joint Settings")]
        [Space(10)]
        public Vector3 currentEulerJointRotation;

        [HideInInspector] public float currentStrength;
        public float currentXNormalizedRot;
        public float currentYNormalizedRot;
        public float currentZNormalizedRot;

        [Header("Other Debug Info")]
        [Space(10)]
        public Vector3 currentJointForce;

        // public float currentJointForceSqrMag;
        // public Vector3 currentJointTorque;
        // public float currentJointTorqueSqrMag;
        // public AnimationCurve jointForceCurve = new AnimationCurve();
        // public AnimationCurve jointTorqueCurve = new AnimationCurve();


        public void Reset(BodyPart bp)
        {
            bp.joint.jointPosition = startingPos;
            bp.joint.jointAcceleration = new ArticulationReducedSpace(0f, 0f, 0f);
            bp.joint.jointForce = new ArticulationReducedSpace(0f, 0f, 0f);
            bp.joint.jointVelocity = new ArticulationReducedSpace(0f, 0f, 0f);
            
            bp.joint.velocity = Vector3.zero;
            bp.joint.angularVelocity = Vector3.zero;
            
            if (bp.groundContact)
            {
                bp.groundContact.touchingGround = false;
            }

            if (bp.targetContact)
            {
                bp.targetContact.touchingTarget = false;
            }
        }

        public void SetJointTargetRotation(float y, float z, float x)
        {
            y = (y + 1f) * 0.5f;
            z = (z + 1f) * 0.5f;
            x = (x + 1f) * 0.5f;

            var yRot = Mathf.Lerp(joint.yDrive.lowerLimit, joint.yDrive.upperLimit, y);
            var zRot = Mathf.Lerp(joint.zDrive.lowerLimit, joint.zDrive.upperLimit, z);
            var xRot = Mathf.Lerp(joint.xDrive.lowerLimit, joint.xDrive.upperLimit, x);

            currentYNormalizedRot = Mathf.InverseLerp(joint.yDrive.lowerLimit, joint.yDrive.upperLimit, yRot);
            currentZNormalizedRot = Mathf.InverseLerp(joint.zDrive.lowerLimit, joint.zDrive.upperLimit, zRot);
            currentXNormalizedRot = Mathf.InverseLerp(joint.xDrive.lowerLimit, joint.yDrive.upperLimit, xRot);

            var drive = joint.yDrive;
            drive.target = yRot;
            joint.yDrive = drive;

            drive = joint.zDrive;
            drive.target = zRot;
            joint.zDrive = drive;

            drive = joint.xDrive;
            drive.target = xRot;
            joint.xDrive = drive;

            currentEulerJointRotation = new Vector3(xRot, yRot, zRot);
        }

        public void SetJointStrength(float strength)
        {
            var rawVal = (strength + 1f) * 0.5f * thisJdController.jointForceLimit;

            var drive = joint.yDrive;
            drive.forceLimit = rawVal;
            joint.yDrive = drive;

            drive = joint.zDrive;
            drive.forceLimit = rawVal;
            joint.zDrive = drive;

            drive = joint.xDrive;
            drive.forceLimit = rawVal;
            joint.xDrive = drive;
            
            currentStrength = rawVal;
        }
    }

    public class JointDriveController : MonoBehaviour
    {
        [Header("Joint Drive Settings")]
        [Space(10)]
        public float jointStiffness;
        public float jointDamping;
        public float jointForceLimit;
        float m_FacingDot;

        [HideInInspector] public Dictionary<Transform, BodyPart> bodyPartsDict = new Dictionary<Transform, BodyPart>();

        [HideInInspector] public List<BodyPart> bodyPartsList = new List<BodyPart>();
        const float k_MaxAngularVelocity = 50.0f;

        public void SetupBodyPart(Transform t)
        {
            var bp = new BodyPart
            {
                joint = t.GetComponent<ArticulationBody>(),
                startingPos = t.GetComponent<ArticulationBody>().jointPosition
            };
            bp.joint.maxAngularVelocity = k_MaxAngularVelocity;

            bp.groundContact = t.GetComponent<GroundContact>();
            if(!bp.groundContact) {
                bp.groundContact = t.gameObject.AddComponent<GroundContact>();
                bp.groundContact.agent = gameObject.GetComponent<AttackAgent>();
            } else {
                bp.groundContact.agent = gameObject.GetComponent<AttackAgent>();
            }

            if (bp.joint) {
                var drive = bp.joint.yDrive;
                drive.stiffness = jointStiffness;
                drive.damping = jointDamping;
                drive.forceLimit = jointForceLimit;
                bp.joint.yDrive = drive;

                drive = bp.joint.zDrive;
                drive.stiffness = jointStiffness;
                drive.damping = jointDamping;
                drive.forceLimit = jointForceLimit;
                bp.joint.zDrive = drive;

                drive = bp.joint.xDrive;
                drive.stiffness = jointStiffness;
                drive.damping = jointDamping;
                drive.forceLimit = jointForceLimit;
                bp.joint.xDrive = drive;
            }

            bp.thisJdController = this;
            bodyPartsDict.Add(t, bp);
            bodyPartsList.Add(bp);
        }

        /*public void GetCurrentJointForces()
        {
            foreach (var bodyPart in bodyPartsDict.Values) {
                if (bodyPart.joint) {
                    bodyPart.currentJointForce = new Vector3(
                            bodyPart.joint.jointForce[0],
                            bodyPart.joint.jointForce[1],
                            bodyPart.joint.jointForce[2]);
                    bodyPart.currentJointForceSqrMag = bodyPart.currentJointForce.magnitude;
                    if (Application.isEditor) {
                        if (bodyPart.jointForceCurve.length > 1000) {
                            bodyPart.jointForceCurve = new AnimationCurve();
                        }
                        if (bodyPart.jointTorqueCurve.length > 1000) {
                            bodyPart.jointTorqueCurve = new AnimationCurve();
                        }
                    }

                    bodyPart.jointForceCurve.AddKey(Time.time, bodyPart.currentJointForceSqrMag); // 0 -> bodyPart.currentJointForceSqrMag
                    bodyPart.jointTorqueCurve.AddKey(Time.time, bodyPart.currentJointTorqueSqrMag); // 0 -> bodyPart.currentJointTorqueSqrMag
                }
            }
        }*/
    }
}