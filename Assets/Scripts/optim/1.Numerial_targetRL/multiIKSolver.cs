using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class multiIKSolver : MonoBehaviour
{

    //public Transform target;
    //public Transform shoulder;
    //public Transform ee;
    public enum Variable { Euler, RotVec };
    public Variable variable = Variable.Euler;
    public bool useConstraints = false;

    Component[] arm_optims;
    Component[] leg_optims;

    // Update is called once per frame
    private void Start()
    {
        arm_optims = gameObject.GetComponents<arm_optim>();
        leg_optims = gameObject.GetComponents<leg_optim>();

    }
    void Update()
    {
        
        if (variable == Variable.Euler)
        {
            // body optimization
            //gameObject.GetComponent<upperbody_optim>().Solve_body_IK_RotVec();        
            gameObject.GetComponent<upperbody_optim>().Solve_body_IK_Euler();

            // limb_optimization
            foreach (Component arm_optim in arm_optims)
            {
                arm_optim.SendMessage("Solve_limb_IK_Euler", useConstraints);
            }

            foreach (Component leg_opim in leg_optims)
            {
                leg_opim.SendMessage("Solve_limb_IK_Euler", useConstraints);
            }

        }
        else if (variable == Variable.RotVec)
        {
            // body optimization
            //gameObject.GetComponent<upperbody_optim>().Solve_body_IK_RotVec();        
            gameObject.GetComponent<upperbody_optim>().Solve_body_IK_RotVec();

            // limb_optimization

            foreach (Component arm_optim in arm_optims)
            {
                arm_optim.SendMessage("Solve_limb_IK_RotVec");
            }

            foreach (Component leg_opim in leg_optims)
            {
                leg_opim.SendMessage("Solve_limb_IK_RotVec");
            }

        }
    }
}
