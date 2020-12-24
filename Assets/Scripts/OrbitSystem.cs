using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

[System.Serializable]
public struct OrbitElements : IComponentData {
    /// <summary> a [m] </summary>
    public double SemiMajorAxis;

    /// <summary> e [1] </summary>
    public double Eccentricity;

    /// <summary> ω [rad] </summary>
    public double ArgumentOfPeriapsis;
    /// <summary> Ω [rad] </summary>
    public double LongitudeOfAscendingNode;

    /// <summary> i [rad] </summary>
    public double Inclination;

    /// <summary> M_0 [rad] at epoch t_0 </summary>
    public double EpochAnomaly;

    /// <summary> µ = GM [m^3.kg^-1·s^-2] </summary>
    public double GravitationalParameter;

    public Entity Parent;
}

[System.Serializable]
public struct OrbitSate : IComponentData {
    public Entity Parent;
    public double3 Position;
    public double3 Velocity;
}

[System.Serializable]
public struct Body : IComponentData {
    public double Mass;
}

public class OrbitSystem : SystemBase
{
    private double simTime = 0;

    /// <summary> G [m^3.kg^-1·s^-2] </summary>
    private const double GRAV_CONST = 6.67408e-11;
    private const double TWO_PI = 2d * math.PI_DBL;

    protected override void OnUpdate() {

        simTime += Time.DeltaTime;

        var currentTime = simTime;

        Entities
            .WithName("Update_Orbiter_State_Vectors")
            .WithAll<Body, OrbitElements, OrbitSate>()
            .ForEach(
                (ref OrbitSate os, in OrbitElements oe) => {
                    // update sate vectors

                    // these parameter could be calculated only once per main body
                    double pstvEccentSqrt = math.sqrt(1 + oe.Eccentricity);
                    double ngtvEccentSqrt = math.sqrt(1 - oe.Eccentricity);
                    double gravParamSemiMajAxis = oe.GravitationalParameter * oe.SemiMajorAxis;
                    double eccentricitySquared = oe.Eccentricity * oe.Eccentricity;


                    // 1- Mean Anomaly Mt [rad]
                    double meanAnomaly = oe.EpochAnomaly + currentTime * math.sqrt(oe.GravitationalParameter / math.pow(oe.SemiMajorAxis, 3));
                    meanAnomaly %= TWO_PI;


                    // 2- Eccentric Anomaly Et
                    // Solve Kepler's Equation M(t) = E(t) - e sin E
                    // using Kepler's method
                    // int maxIter = 50;
                    // double eccentricAnomaly = meanAnomaly;
                    // for(int i = 0; i < maxIter; i++) {
                    //     eccentricAnomaly = meanAnomaly + elements.Eccentricity * math.sin(eccentricAnomaly);
                    // }
                    
                    // using Newton's method
                    int iter = 0;
                    int maxIter = 50;
                    double delta = 0.00000000001; // 10 0, could go to 14

                    double eccentricAnomaly = meanAnomaly;
                    double eccentricAnomalyPrime = eccentricAnomaly - oe.Eccentricity * math.sin(eccentricAnomaly) - meanAnomaly;

                    while (math.abs(eccentricAnomalyPrime) > delta && iter < maxIter) {
                        eccentricAnomaly = eccentricAnomaly - eccentricAnomalyPrime / (1 - oe.Eccentricity * math.cos(eccentricAnomaly));
                        eccentricAnomalyPrime = eccentricAnomaly - oe.Eccentricity * math.sin(eccentricAnomaly) - meanAnomaly;
                        iter++;
                    }


                    // 3- True Anomaly vt [rad]
                    double trueAnomaly = 2.0 * math.atan2
                    (
                        pstvEccentSqrt * math.sin(eccentricAnomaly / 2.0),
                        ngtvEccentSqrt * math.cos(eccentricAnomaly / 2.0)
                    );


                    // 4 - distance to center at t: rt
                    double distance = oe.SemiMajorAxis * (1 - oe.Eccentricity * math.cos(eccentricAnomaly));


                    // 5 - position and veloctiy in the orbital frame (z-axis perpendicular to orbital plane, x-axis pointing to periapsis of the orbit):
                    double cosTrueAno = math.cos(trueAnomaly);
                    double sinTrueAno = math.sin(trueAnomaly);
                    double cosEccAno = math.cos(eccentricAnomaly);
                    double sinEccAno = math.sin(eccentricAnomaly);

                    double3 pos_o = new double3
                    (
                        cosTrueAno,
                        sinTrueAno,
                        0
                    ) * distance;

                    double3 vel_o = new double3
                    (
                        -sinEccAno,
                        math.sqrt(1 - eccentricitySquared) * cosEccAno,
                        0
                    ) * math.sqrt(gravParamSemiMajAxis) / distance;


                    // 6 - position and velocity in orbital parent frame
                    // WARNING Unity coordinates and Heliocentric coordinates have z and y axis swapped
                    double cosAsc = math.cos(oe.LongitudeOfAscendingNode);
                    double cosPeri = math.cos(oe.ArgumentOfPeriapsis);
                    double cosIncl = math.cos(oe.Inclination);
                    double sinAsc = math.sin(oe.LongitudeOfAscendingNode);
                    double sinPeri = math.sin(oe.ArgumentOfPeriapsis);
                    double sinIncl = math.sin(oe.Inclination);

                    double x_a = math.cos(oe.LongitudeOfAscendingNode) * math.cos(oe.ArgumentOfPeriapsis) - math.sin(oe.LongitudeOfAscendingNode) * math.cos(oe.Inclination) * math.sin(oe.ArgumentOfPeriapsis) ;
                    double y_a = math.sin(oe.LongitudeOfAscendingNode) * math.cos(oe.ArgumentOfPeriapsis) + math.cos(oe.LongitudeOfAscendingNode) * math.cos(oe.Inclination) * math.sin(oe.ArgumentOfPeriapsis);
                    double z_a = math.sin(oe.Inclination) * math.sin(oe.ArgumentOfPeriapsis);


                    double x_b = math.cos(oe.LongitudeOfAscendingNode) * math.sin(oe.ArgumentOfPeriapsis) + math.sin(oe.LongitudeOfAscendingNode) * math.cos(oe.Inclination) * math.cos(oe.ArgumentOfPeriapsis) ;
                    double y_b = math.cos(oe.LongitudeOfAscendingNode) * math.cos(oe.Inclination) * math.cos(oe.ArgumentOfPeriapsis) - math.sin(oe.LongitudeOfAscendingNode) * math.sin(oe.ArgumentOfPeriapsis);
                    double z_b =  math.sin(oe.Inclination) * math.cos(oe.ArgumentOfPeriapsis);

                    // don't forget, Y and Z are reversed for Unity
                    os.Position = new double3
                    (
                        pos_o.x * x_a - pos_o.y * x_b,
                        pos_o.x * z_b + pos_o.y * z_b,
                        pos_o.x * y_a + pos_o.y * y_b
                    );

                    os.Velocity = new double3
                    (
                        vel_o.x * x_a - vel_o.y * x_b,
                        vel_o.x * z_b + vel_o.y * z_b,
                        vel_o.x * y_a + vel_o.y * y_b
                    );

                }
            )
            .ScheduleParallel();

    }


    
}
