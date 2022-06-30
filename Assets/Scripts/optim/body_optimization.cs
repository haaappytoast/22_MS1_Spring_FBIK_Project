using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using System;

//[DllImport("alglib318gpl_net5")]
//public static extern void "함수이름"


public class body_optimization : MonoBehaviour
{

    public Transform[] Ts;
    public Transform shoulder;

    public Transform target;
    public Transform endEffector;

    private float boneLength = 0.7294757f;


    void Start()
    {
        Ts = new Transform[4];

        //initialize robot body parts
        Ts[0] = GameObject.Find("mixamorig:Hips").transform;                // hinge joint
        Ts[1] = GameObject.Find("mixamorig:Spine").transform;               // ball joint
        Ts[2] = GameObject.Find("mixamorig:Spine1").transform;              // ball joint
        Ts[3] = GameObject.Find("mixamorig:Spine2").transform;              // ball joint

        boneLength = (endEffector.position - shoulder.position).magnitude;
    }


    //set pose with ball joint
    void SetPose(double[] x, Transform Ts)
    {
        Quaternion localRotation = Quaternion.Euler((float)x[0], (float)x[1], (float)x[2]);
        Ts.localRotation *= localRotation;

    }

    //set pose with hinge joint
    void SetPose(double x, Vector3 axis, Transform Ts)
    {
        Quaternion localRotation = Quaternion.AngleAxis((float)x, axis);

        Ts.localRotation *= localRotation;
    }

    // ref double loss를 minimize함 
    // 일단, Ts[0], RightArm을 optimize 해볼까?
    // radian
    void IK_Loss_Function(double[] x, ref double loss, object obj)
    {
        var tar = (obj as body_optimization).target;

        // Definition of x
        // x[0] : Hips Y-axis Euler Rotation
        // x[1], [2], [3] : Spine XYZ-axis Euler Rotation
        // x[4], [5], [6] : Spine1 XYZ-axis Euler Rotation
        // x[7], [8], [9] : Spine2 XYZ-axis Euler Rotation

        Quaternion[] localRotation = new Quaternion[4];
        localRotation[0] = Quaternion.Euler(0, (float)x[0], 0);
        localRotation[1] = Quaternion.Euler((float)x[1], (float)x[2], (float)x[3]);
        localRotation[2] = Quaternion.Euler((float)x[4], (float)x[5], (float)x[6]);
        localRotation[3] = Quaternion.Euler((float)x[7], (float)x[8], (float)x[9]);

        Quaternion[] dRotation = new Quaternion[4];
        dRotation[0] = Ts[0].rotation * localRotation[0];
        dRotation[1] = Ts[1].localRotation * localRotation[1];
        dRotation[2] = Ts[2].localRotation * localRotation[2];
        dRotation[3] = Ts[3].localRotation * localRotation[3];

        Vector3 Ts1_globalPosition = Ts[0].position + dRotation[0] * Ts[1].localPosition;                                                                           // localPos to globalPos of Ts[1]  spine
        Vector3 Ts2_globalPosition = Ts1_globalPosition + dRotation[0] * dRotation[1] * Ts[2].localPosition;                                                        // localPos to globalPos of Ts[2]  spine1
        Vector3 Ts3_globalPosition = Ts2_globalPosition + dRotation[0] * dRotation[1] * dRotation[2] * Ts[3].localPosition;                                         // localPos to globalPos of Ts[3]  spine2
        Vector3 shoulderPosition = Ts3_globalPosition + dRotation[0] * dRotation[1] * dRotation[2] * dRotation[3] * shoulder.localPosition;                         // localPos to globalPos of EE (End Effector)

        // This is distance.
        //float distance = (Ts[1].position - tar.position).sqrMagnitude;
        float distance = (shoulderPosition - tar.position).magnitude;

        //// Loss
        //loss = (distance - boneLength) * (distance - boneLength);
        
        // shoulder와 target position의 길이 차이가 bone length보다 멀다면
        if((distance - boneLength) > 0.01f)
        {
            loss = (distance - boneLength) * (distance - boneLength);
        }
        // shoulder와 target position의 길이 차이가 bone length보다 가까울 때, (즉 optimization하지 않아도 되는 것)
        else
        {
            loss = 0.0f;
        }

        // regularizer --> smoothing 효과가 나네?!
        double reg = 0.0f;

        foreach (double theta in x)
        {
            reg += theta * theta;
        }

        loss += 0.005f * reg;
    }

    public void Solve_body_IK()
    {
        // initial guess
        double[] x = new double[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

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
        alglib.minlbfgsoptimize(state, IK_Loss_Function, null, this);       // min lbfgs optimize
        alglib.minlbfgsresults(state, out x, out rep);


        // Hips: hinge joint
        SetPose(x[0], Vector3.up, Ts[0]);

        // Spine: ball joint
        SetPose(new double[] { x[1], x[2], x[3] }, Ts[1]);

        // Spine1: ball joint
        SetPose(new double[] { x[4], x[5], x[6] }, Ts[2]);

        // Spine2: ball joint
        SetPose(new double[] { x[7], x[8], x[9] }, Ts[3]);
    }




    // Update is called once per frame
    void Update()
    {
        //Solve();
        //Solve_body_IK();
    }
}
