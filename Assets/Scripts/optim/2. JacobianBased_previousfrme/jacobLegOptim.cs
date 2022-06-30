using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using System;

//[DllImport("alglib318gpl_net5")]
//public static extern void "함수이름"


public class jacobLegOptim : MonoBehaviour
{

    public Transform[] Ts;
    public Transform endEffector;
    public Transform target;


    void SetPose(double[] x, Transform Ts)
    {
        Quaternion dlocalRotation = Quaternion.identity;

        Vector3 aaxis = new Vector3((float)x[0], (float)x[1], (float)x[2]) * Mathf.Rad2Deg;

        dlocalRotation = Quaternion.AngleAxis(aaxis.magnitude, aaxis.normalized);

        // Tpose frame 기준
        Quaternion localRot = Ts.localRotation * dlocalRotation;

        Ts.localRotation = localRot;
    }

    void ToAngleAxis(Quaternion quat, out float angle, out Vector3 axis, int trf_idx)
    {

        angle = 2 * Mathf.Acos(quat.w);

        float qw2 = quat.w * quat.w;

        float x = (float)quat.x / (float)Mathf.Sqrt(1 - qw2);
        float y = (float)quat.y / (float)Mathf.Sqrt(1 - qw2);
        float z = (float)quat.z / (float)Mathf.Sqrt(1 - qw2);

        axis = new Vector3(x, y, z);
    }

    Vector3 get_dp_dx(int trf_idx, Vector3 dist, Vector3 xh, Quaternion expInv)
    {
        Quaternion expxh = Quaternion.AngleAxis(xh.magnitude, xh.normalized);

        Quaternion expy = expInv * expxh;

        // radian
        float angle;
        Vector3 axis;

        ToAngleAxis(expy, out angle, out axis, trf_idx);

        Vector3 worldAxis = Ts[trf_idx].rotation * axis;

        Vector3 dp_dx = Vector3.Cross(worldAxis.normalized, dist);

        dp_dx = angle * dp_dx;
        return dp_dx;
    }

    float loss_func(ref double[] x)
    {

        // Definition of x
        // x[0], [1], [2] : Right Arm's XYZ-axis RotVec Rotation

        Quaternion[] localRotation = new Quaternion[3];
        Vector3 aaxis0 = new Vector3((float)x[0], (float)x[1], (float)x[2]) * Mathf.Rad2Deg;
        Vector3 aaxis1 = new Vector3((float)x[4], (float)x[5], (float)x[6]) * Mathf.Rad2Deg;

        //localRotation[0] = Quaternion.AngleAxis(aaxis0.magnitude * Mathf.Rad2Deg, aaxis0.normalized);
        localRotation[0] = Quaternion.AngleAxis(aaxis0.magnitude, aaxis0.normalized);
        localRotation[1] = Quaternion.AngleAxis((float)x[3] * Mathf.Rad2Deg, Vector3.right);

        Quaternion[] dRotation = new Quaternion[3];
        dRotation[0] = Ts[0].rotation * localRotation[0];           // right arm rotation
        dRotation[1] = Ts[1].localRotation * localRotation[1];      // right forearm
        dRotation[2] = Ts[2].localRotation * localRotation[1];      // right forearm

        Vector3 Ts1_globalPosition = Ts[0].position + dRotation[0] * Ts[1].localPosition;                                       // localPos to globalPos of Ts[1]
        Vector3 Ts2_globalPosition = Ts1_globalPosition + dRotation[0] * dRotation[1] * Ts[2].localPosition;                        // localPos to globalPos of Ts[2]
        Vector3 EEglobalPosition = Ts2_globalPosition + dRotation[0] * dRotation[1] * dRotation[2] * endEffector.localPosition;     // localPos to globalPos of endEffector

        float distance = (EEglobalPosition - target.position).sqrMagnitude;

        return distance;

    }

    Vector3 get_dL_dp()
    {
        Vector3 curT = endEffector.position;        // initial frame: Tpose
        Vector3 tarT = target.position;
        Vector3 dist = (curT - tarT);


        Vector3 dL_dp = 2 * dist;

        return dL_dp;
    }
    double[] get_dL_dx(int trf_idx, double[] x, Vector3 dL_dp)
    {
        double[] grad = new double[] { 0, 0, 0 };

        // world coord
        Vector3 b = (endEffector.position - Ts[trf_idx].position);

        Vector3 aaxis = new Vector3((float)x[0], (float)x[1], (float)x[2]) * Mathf.Rad2Deg;

        //var expx = Quaternion.AngleAxis(aaxis.magnitude * Mathf.Rad2Deg, aaxis.normalized);
        var expx = Quaternion.AngleAxis(aaxis.magnitude, aaxis.normalized);
        var expInv = Quaternion.Inverse(expx);

        // expInv(x + h) = exp(x) * exp(y)
        // exp(y) = exp(x).inv * exp(x + h); h = [1, 0, 0] or [0, 1, 0] or [0, 0, 1]
        // xh = x + h
        for (int rot_idx = 0; rot_idx < 3; ++rot_idx)
        {
            // 계속 initialize
            Vector3 xh = aaxis;

            xh[rot_idx] += 1.0f;

            Vector3 dp_dx = get_dp_dx(trf_idx, b, xh, expInv);

            // get dL/dX
            float dL_dx = Vector3.Dot(dL_dp, dp_dx);

            grad[rot_idx] = dL_dx;
        }

        return grad;

    }

    double get_dL_dx(int trf_idx, double x, Vector3 dL_dp)
    {
        double grad = 0;

        // world coord
        Vector3 b = (endEffector.position - Ts[trf_idx].position);

        Vector3 aaxis = new Vector3(-(float)x, 0, 0) * Mathf.Rad2Deg;

        //var expx = Quaternion.AngleAxis(aaxis.magnitude * Mathf.Rad2Deg, aaxis.normalized);
        var expx = Quaternion.AngleAxis(aaxis.magnitude, aaxis.normalized);
        var expInv = Quaternion.Inverse(expx);

        // expInv(x + h) = exp(x) * exp(y)
        // exp(y) = exp(x).inv * exp(x + h); h = [1, 0, 0] or [0, 1, 0] or [0, 0, 1]
        // xh = x + h
        Vector3 xh = aaxis;
        xh[0] += 1.0f;
        Vector3 dp_dx = get_dp_dx(trf_idx, b, xh, expInv);

        // get dL/dX
        float dL_dx = Vector3.Dot(dL_dp, dp_dx);

        grad = dL_dx;

        return grad;
    }
    void Calc_gradient(double[] x, ref double func, double[] grad, object obj)
    {
        // function

        // This is distance.

        func = loss_func(ref x);

        // get dL/dP
        Vector3 dL_dp = get_dL_dp();

        // get dP/dX
        double[] temp;

        // 1. 1st joint
        temp = get_dL_dx(0, new double[] { x[0], x[1], x[2] }, dL_dp);
        for (int i = 0; i < 3; ++i)
        {
            grad[i] = temp[i];
        }

        // 2. 2nd joint
        grad[3] = get_dL_dx(1, x[3], dL_dp);

        // 3. 3rd joint
        temp = get_dL_dx(2, new double[] { x[4], x[5], x[6] }, dL_dp);
        for (int i = 0; i < 3; ++i)
        {
            grad[i + 4] = temp[i];
        }

    }


    // RotVec - Analytic
    public void Solve_limb_IK_RotVec_grad()
    {
        // initial guess (in rad)
        double[] x = new double[] { 0, 0, 0, 0, 0, 0, 0 };

        double epsg = 0;
        double epsf = 0;
        double epsx = 0;
        double stpmax = 0.1;
        int maxits = 0;

        alglib.minlbfgsstate state;
        alglib.minlbfgsreport rep;

        // Analytical 
        alglib.minlbfgscreate(1, x, out state);                         // min lbfgs create
        alglib.minlbfgssetcond(state, epsg, epsf, epsx, maxits);        // min lbfgs set condition (sets stopping conditions for L-BFGS optimization algorithm)
        alglib.minlbfgssetstpmax(state, stpmax);

        alglib.minlbfgsoptimize(state, Calc_gradient, null, this);      // min lbfgs optimize
        alglib.minlbfgsresults(state, out x, out rep);

        // ball joint
        SetPose(new double[] { x[0], x[1], x[2] }, Ts[0]);

        // hinge joint
        SetPose(new double[] { x[3], 0 , 0 }, Ts[1]);
        ////SetPose(x[3], Vector3.up, Ts[1]);

        // ball joint
        SetPose(new double[] { x[4], x[5], x[6] }, Ts[2]);
    }

    void Update()
    {
        //Solve_limb_IK_RotVec_grad();
    }
}
