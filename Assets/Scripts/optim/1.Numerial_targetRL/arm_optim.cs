using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using System;

//[DllImport("alglib318gpl_net5")]
//public static extern void "함수이름"


public class arm_optim : MonoBehaviour
{


    bool contraints = false;

    public Transform[] Ts;
    public Transform endEffector;
    public Transform target;

    public enum Side { right, left };

    public Side side = Side.right;

    float[] FminLimit = new float[] { -136f, -136f };
    float[] FmaxLimit = new float[] { 0.0f, 0.0f };
    void Start()
    {
        if(side == Side.left)
        {
            FminLimit = new float[] { 0.0f, 0.0f };
            FmaxLimit = new float[] { 136f, 136f };
        }

    }

    void SetPose(double[] x, Transform Ts, string rep)
    {
        Quaternion localRotation = Quaternion.identity;
        if (rep == "Euler")
        {
            localRotation = Quaternion.Euler((float)x[0], (float)x[1], (float)x[2]);
        }
        if (rep == "RotVec")
        {

            Vector3 aaxis = new Vector3((float)x[0], (float)x[1], (float)x[2]);
            localRotation = Quaternion.AngleAxis(aaxis.magnitude, aaxis.normalized);
        }

        Ts.localRotation *= localRotation;
    }

    void SetPose(double x, Vector3 axis, Transform Ts)
    {
        Quaternion localRotation = Quaternion.AngleAxis((float)x, axis);

        Ts.localRotation *= localRotation;
    }

    // ref double loss를 minimize함 
    // 1. Euler - Numerical
    void IK_Loss_Function_Euler(double[] x, ref double loss, object obj)
    {
        var tar = (obj as arm_optim).target;

        // Definition of x
        // x[0], [1], [2] : Right Arm's XYZ-axis Euler Rotation
        // x[3]           : Right ForeArm's Y-axis Euler Rotation
        // x[4], [5], [6] : Right Hand's XYZ-axis Euler Rotation

        Quaternion[] localRotation = new Quaternion[3];
        localRotation[0] = Quaternion.Euler((float)x[0], (float)x[1], (float)x[2]);
        localRotation[1] = Quaternion.Euler(0, (float)x[3], 0);
        localRotation[2] = Quaternion.Euler((float)x[4], (float)x[5], (float)x[6]);

        Quaternion[] dRotation = new Quaternion[3];
        dRotation[0] = Ts[0].rotation * localRotation[0];           // right arm
        dRotation[1] = Ts[1].localRotation * localRotation[1];      // right forearm
        dRotation[2] = Ts[2].localRotation * localRotation[2];      // right hand

        Vector3 Ts1_globalPosition = Ts[0].position + dRotation[0] * Ts[1].localPosition;                                           // localPos to globalPos of Ts[1]
        Vector3 Ts2_globalPosition = Ts1_globalPosition + dRotation[0] * dRotation[1] * Ts[2].localPosition;                        // localPos to globalPos of Ts[2]
        Vector3 EEglobalPosition = Ts2_globalPosition + dRotation[0] * dRotation[1] * dRotation[2] * endEffector.localPosition;     // localPos to globalPos of endEffector


        // This is distance.
        float distance = (EEglobalPosition - tar.position).sqrMagnitude;

        // Loss
        loss = distance;

        // constraints: joint limits

        if(contraints)
        {
            for (int i = 1; i < 2; i++)
            {
                float jntAngle = (float)dRotation[i].eulerAngles.y;
                if (jntAngle > 180)
                {
                    jntAngle = -360.0f + jntAngle;
                }

                float FArmJointLimit_Loss = 0.0f;

                //// if (FArmJointLimit > minimum ) ->  그대로
                if ((jntAngle - FminLimit[i] >= 0) && (jntAngle - FmaxLimit[i] <= 0))
                {
                    FArmJointLimit_Loss = 0.0f;
                }
                else if (jntAngle - FminLimit[i] < 0)
                {
                    FArmJointLimit_Loss = (jntAngle - FminLimit[i]) * (jntAngle - FminLimit[i]);
                }
                else if (jntAngle - FmaxLimit[i] > 0)
                {
                    FArmJointLimit_Loss = (jntAngle - FmaxLimit[i]) * (jntAngle - FmaxLimit[i]);
                }

                loss += FArmJointLimit_Loss;
                //Debug.Log(FArmJointLimit_Loss);

            }

        }



    }

    public void Solve_limb_IK_Euler(bool useConstraints)
    {
        contraints = useConstraints;

        // initial guess (in degrees)
        double[] x = new double[] { 0, 0, 0, 0, 0, 0, 0 };

        double epsg = 0;
        double epsf = 0;
        double epsx = 0;
        double diffstep = 1.0e-2;
        int maxits = 0;

        alglib.minlbfgsstate state;
        alglib.minlbfgsreport rep;

        // Numerical 
        alglib.minlbfgscreatef(1, x, diffstep, out state);              // min lbfgs create
        alglib.minlbfgssetcond(state, epsg, epsf, epsx, maxits);        // min lbfgs set condition (sets stopping conditions for L-BFGS optimization algorithm)
        alglib.minlbfgsoptimize(state, IK_Loss_Function_Euler, null, this);       // min lbfgs optimize
        alglib.minlbfgsresults(state, out x, out rep);

        //Debug.Log(rep.terminationtype);
        //Debug.Log(alglib.ap.format(x, 2));

        // ball joint
        SetPose(new double[] { x[0], x[1], x[2] }, Ts[0], "Euler");

        // hinge joint
        SetPose(x[3], Vector3.up, Ts[1]);

        // ball joint
        SetPose(new double[] { x[4], x[5], x[6] }, Ts[2], "Euler");
    }

    // 2. RotVec - Numerical
    void IK_Loss_Function_RotVec(double[] x, ref double loss, object obj)
    {
        var tar = (obj as arm_optim).target;

        // Definition of x
        // x[0], [1], [2] : Right Arm's XYZ-axis Euler Rotation
        // x[3]           : Right ForeArm's Y-axis Euler Rotation
        // x[4], [5], [6] : Right Hand's XYZ-axis Euler Rotation

        Quaternion[] localRotation = new Quaternion[3];
        Vector3 aaxis0 = new Vector3((float)x[0], (float)x[1], (float)x[2]);
        Vector3 aaxis1 = new Vector3((float)x[6], (float)x[7], (float)x[8]);

        localRotation[0] = Quaternion.AngleAxis(aaxis0.magnitude, aaxis0.normalized);
        localRotation[1] = Quaternion.AngleAxis((float)x[4], Vector3.up);
        localRotation[2] = Quaternion.AngleAxis(aaxis1.magnitude, aaxis1.normalized);

        Quaternion[] dRotation = new Quaternion[3];
        dRotation[0] = Ts[0].rotation * localRotation[0];           // right arm
        dRotation[1] = Ts[1].localRotation * localRotation[1];      // right forearm
        dRotation[2] = Ts[2].localRotation * localRotation[2];      // right hand

        Vector3 Ts1_globalPosition = Ts[0].position + dRotation[0] * Ts[1].localPosition;                                           // localPos to globalPos of Ts[1]
        Vector3 Ts2_globalPosition = Ts1_globalPosition + dRotation[0] * dRotation[1] * Ts[2].localPosition;                        // localPos to globalPos of Ts[2]
        Vector3 EEglobalPosition = Ts2_globalPosition + dRotation[0] * dRotation[1] * dRotation[2] * endEffector.localPosition;     // localPos to globalPos of endEffector

        // This is distance.
        float distance = (EEglobalPosition - tar.position).sqrMagnitude;

        // Loss
        loss = distance;
    }

    public void Solve_limb_IK_RotVec()
    {
        // initial guess (in degrees)
        double[] x = new double[] { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        double epsg = 0;
        double epsf = 0;
        double epsx = 0;
        double diffstep = 1.0e-2;
        int maxits = 0;

        alglib.minlbfgsstate state;
        alglib.minlbfgsreport rep;

        // Numerical 
        alglib.minlbfgscreatef(1, x, diffstep, out state);              // min lbfgs create
        alglib.minlbfgssetcond(state, epsg, epsf, epsx, maxits);        // min lbfgs set condition (sets stopping conditions for L-BFGS optimization algorithm)
        alglib.minlbfgsoptimize(state, IK_Loss_Function_RotVec, null, this);       // min lbfgs optimize
        alglib.minlbfgsresults(state, out x, out rep);

        // ball joint
        SetPose(new double[] { x[0], x[1], x[2] }, Ts[0], "RotVec");

        // hinge joint
        SetPose(new double[] { 0, x[4], 0 }, Ts[1], "RotVec");
        //SetPose(x[3], Vector3.up, Ts[1]);

        // ball joint
        SetPose(new double[] { x[6], x[7], x[8] }, Ts[2], "RotVec");
    }

    // Update is called once per frame
    void Update()
    {
        //Solve();
        //Solve_limb_IK();
    }
}
