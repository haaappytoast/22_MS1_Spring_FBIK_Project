using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IKSolver : MonoBehaviour
{

    public Transform target;
    public Transform shoulder;
    public Transform ee;


    private double boneLength;

    // Start is called before the first frame update
    void Awake()
    {
        boneLength = (shoulder.position - ee.position).magnitude;
    }

    // Update is called once per frame
    void Update()
    {
        // body optimization
        if ((shoulder.position - target.position).magnitude > boneLength) // -> �̹� optimization���� loss�� �������� �� ����!
        {
            gameObject.GetComponent<body_optimization>().Solve_body_IK();
        }

        // limb_optimization
            gameObject.GetComponent<limb_optimization>().Solve_limb_IK_Euler();
    }
}
