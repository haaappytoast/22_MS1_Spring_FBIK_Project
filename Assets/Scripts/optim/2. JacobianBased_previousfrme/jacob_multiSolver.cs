using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class jacob_multiSolver : MonoBehaviour
{

    //public Transform target;
    //public Transform shoulder;
    //public Transform ee;

    Component[] arm_optims;
    Component[] leg_optims;

    // Update is called once per frame
    private void Start()
    {
        arm_optims = gameObject.GetComponents<jacobArmOptim>();
        leg_optims = gameObject.GetComponents<jacobLegOptim>();
    }
    void Update()
    {
        // body optimization
        //gameObject.GetComponent<upperbody_optim>().Solve_body_IK_RotVec();        
        gameObject.GetComponent<upperbody_optim>().Solve_body_IK_RotVec();

        // limb_optimization

        foreach (Component arm_optim in arm_optims)
        {
            arm_optim.SendMessage("Solve_limb_IK_RotVec_grad");
        }
        foreach (Component leg_opim in leg_optims)
        {
            leg_opim.SendMessage("Solve_limb_IK_RotVec_grad");
        }
    }
}
