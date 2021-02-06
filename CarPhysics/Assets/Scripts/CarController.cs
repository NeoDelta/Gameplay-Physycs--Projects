using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using static UnityEngine.InputSystem.InputAction;

struct TorqueRPM
{
    public float torque;
    public float RPM;
}
public class CarController : MonoBehaviour
{
    public Vector3 velocity;
    public Text carSpeedUI;
    public Text carRPMUI;
    public Text carGearUI;
    public MeshRenderer breakLightsRenderer;
    public GameObject cam;

    int currentGear;
    float[] gearRatio;
    TorqueRPM[] engineCurve;
    float differentialRatio;
    float transmissionEffieciency;
    float wheelRadius;
    float wheelMass;
    float carMass = 1439.0f;
    float cDrag = 0.4257f;
    float cRr = 12.8f; // Aprox. 30 times cDrag
    float currentRPM = 1000f;
    float maxRPM = 5900f;
    float minRPM = 1000f;
    float wheelAngularVelocity = 0f;
    float throttleInput = 0.0f;
    float breakInput = 0.0f;
    float steerAngleInput = 0.0f;

    Rigidbody rb;
    void Start()
    {
        rb = this.GetComponent<Rigidbody>();
        gearRatio = new float [] { 2.66f, 1.78f, 1.30f, 1.00f, 0.74f, 0.50f, 2.90f};
        differentialRatio = 3.42f;
        transmissionEffieciency = 0.7f;
        currentGear = 0;

        engineCurve = new TorqueRPM[] { 
            new TorqueRPM { torque = 390, RPM = 1000 },
            new TorqueRPM { torque = 440, RPM = 2000 },
            new TorqueRPM { torque = 460, RPM = 3000 },
            new TorqueRPM { torque = 480, RPM = 4000 },
            new TorqueRPM { torque = 470, RPM = 5000 },
            new TorqueRPM { torque = 390, RPM = 5900 }
        };

        wheelRadius = 0.33f;
        wheelMass = 75f;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        //velocity = rb.velocity;
        float slip;
        
        if (velocity.magnitude == 0) slip = 0;
        else slip = (wheelAngularVelocity * wheelRadius - velocity.magnitude) / velocity.magnitude;

        float tractionForce = 1000f * slip;
        if (tractionForce > 6000f) tractionForce = 6000f;
        float tractionTorque = tractionForce * wheelRadius;

        currentRPM = wheelAngularVelocity * gearRatio[currentGear] * differentialRatio * 60f / (2f * Mathf.PI);
        if (currentRPM < minRPM)
        {
            currentRPM = minRPM;
        }
        else if (currentRPM < 2000f)
        {
            if (currentGear > 0) currentGear -= 1;
        }
        else if (currentRPM > maxRPM)
        {
            currentRPM = maxRPM;
            if(currentGear < 5) currentGear += 1;
        }
        //if (steerAngleInput == 0f) steerAngleInput = 0.00001f;
        float curveRadius = 3f / Mathf.Sin(steerAngleInput);
        Vector3 angularVelocity = Vector3.up * velocity.magnitude / curveRadius;

        float tEngine = throttleInput * LookupTorqueCurve(currentRPM);
        float wheelTorque = tEngine * gearRatio[currentGear] * differentialRatio * transmissionEffieciency;
        float wheelForce = wheelTorque / (1 * wheelRadius);

        float frictionMult = (-Mathf.Abs(Vector3.Dot(this.transform.forward, velocity.normalized)) + 2f) * 5f;
        Vector3 fDrive = this.transform.forward * wheelForce;
        Vector3 fTraction = fDrive;
        Vector3 fDrag = -cDrag * velocity * velocity.magnitude;
        Vector3 fRr = -cRr * velocity * frictionMult;
        Vector3 fBrake = -this.velocity.normalized * 15000f * breakInput;
        if (Vector3.Dot(velocity.normalized, this.transform.forward) < 0.0f) fBrake = Vector3.zero;

        float alpha = Vector3.SignedAngle(velocity, this.transform.forward, Vector3.up);
        float latVel = Mathf.Sin(alpha) * velocity.magnitude;
        float lonVel = Mathf.Cos(alpha) * velocity.magnitude;
        float omega = velocity.magnitude / curveRadius;

        float fLatRear = Mathf.Atan2(latVel, Mathf.Abs(lonVel)) - omega * 1.5f;
        float fLatFront = Mathf.Atan2(latVel, Mathf.Abs(lonVel)) + omega * 1.5f - steerAngleInput * Mathf.Sign(lonVel);

        Vector3 fCornering = -this.transform.right.normalized * (4000f * fLatRear + Mathf.Cos(steerAngleInput) * 4000f * fLatFront);
        if (steerAngleInput <= 0.00001f && steerAngleInput >= -0.00001f) fCornering = Vector3.zero;

        Vector3 fLong = fTraction + fDrag + fRr + fBrake + fCornering;
        Vector3 a = fLong / carMass;

        //if (wheelTorque > 0) tractionTorque *= -1; //¿?
        float breakTorque = fBrake.magnitude * wheelRadius;
        float totalTorque = wheelTorque - 2 * tractionTorque - breakTorque; ;
        float wheelInertia = wheelMass * wheelRadius * wheelRadius / 2;
        float angularAcceleration = totalTorque / wheelInertia;
        wheelAngularVelocity += angularAcceleration * Time.deltaTime;

        velocity += a * Time.deltaTime;
        this.transform.position += velocity * Time.deltaTime;
       // if(velocity.magnitude != 0f) this.transform.rotation = Quaternion.LookRotation(velocity);

        //rb.AddForce(a);
        float latTorque = (4000 * -fLatRear * 1.5f + Mathf.Cos(steerAngleInput) * 4000 * fLatFront * 1.5f) /carMass;
        //rb.AddTorque(latTorque * this.transform.up * Time.deltaTime);
        if (latTorque != 0f) this.transform.Rotate(latTorque/1.25f * Mathf.Rad2Deg * this.transform.up * Time.deltaTime * Time.deltaTime);

        carSpeedUI.text = ((int)(velocity.magnitude * 3.6f)).ToString() + " km/h";
        carRPMUI.text = ((int)currentRPM).ToString() + " rpm";
        carGearUI.text = (currentGear+1).ToString();


        if(velocity.magnitude > 0.01f) cam.transform.rotation = Quaternion.LookRotation(velocity);
        else cam.transform.rotation = Quaternion.LookRotation(this.transform.forward);

        Debug.Log(Mathf.Atan2(latVel, Mathf.Abs(lonVel)));
        Debug.DrawLine(this.transform.position, this.transform.position + fCornering.normalized * 10f);
    }

    private float LookupTorqueCurve(float rpm)
    {
        for(uint i = 1; i < engineCurve.Length; i++)
        {
            if(rpm <= engineCurve[i].RPM)
            {
                float step = (rpm - engineCurve[i-1].RPM) / (engineCurve[i].RPM - engineCurve[i - 1].RPM);
                return Mathf.Lerp(engineCurve[i - 1].torque, engineCurve[i].torque, step);
            }
        }

        return engineCurve[0].torque;
    }

    public void ThrottleInput(CallbackContext context)
    {
        throttleInput = context.ReadValue<float>();
    }

    public void BreakInput(CallbackContext context)
    {
        breakInput = context.ReadValue<float>();

        if (breakInput > 0f) breakLightsRenderer.enabled = true;
        else breakLightsRenderer.enabled = false;
    }

    public void SteerInput(CallbackContext context)
    {
        steerAngleInput = context.ReadValue<Vector2>().x/2.0f;
    }
}
