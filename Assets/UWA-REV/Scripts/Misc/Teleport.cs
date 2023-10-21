using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
// namespace AWSIM
// {
    public class Teleport : MonoBehaviour
    {
        [SerializeField] GameObject SPH;
        [SerializeField] GameObject SP1;
        [SerializeField] GameObject SP2;
        [SerializeField] GameObject SP3;

        Vector3 TPH;
        Vector3 TP1;
        Vector3 TP2;
        Vector3 TP3;

        // Start is called before the first frame update
        void Start()
        {
            TPH = SPH.transform.position;
            TP1 = SP1.transform.position;
            TP2 = SP2.transform.position;
            TP3 = SP3.transform.position;
        }

        // Update is called once per frame
        void Update()
        {
            
        }

        public void teleportVehicleHome()
        {
            gameObject.transform.position = TPH;
            Debug.Log("TELEPORT Home");
        }
        public void teleportVehicle1()
        {
            gameObject.transform.position = TP1;
            Debug.Log("TELEPORT 1");
        }
        public void teleportVehicle2()
        {
            gameObject.transform.position = TP2;
            Debug.Log("TELEPORT 2");
        }
        public void teleportVehicle3()
        {
            gameObject.transform.position = TP3;
            Debug.Log("TELEPORT 3");
        }
    }
// }
