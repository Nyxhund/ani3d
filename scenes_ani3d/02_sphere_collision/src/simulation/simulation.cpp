#include "simulation.hpp"

using namespace cgp;

struct plane
{
    vec3 n;
    vec3 origin;
};

struct plane planes[6] = {
    { {1, 0, 0}, {-1, -1, -1} },
    { {0, 1, 0}, {-1, -1, -1} },
    { {0, 0, 1}, {-1, -1, -1} },
    { {-1, 0, 0}, {1, 1, 1} },
    { {0, -1, 0}, {1, 1, 1} },
    { {0, 0, -1}, {1, 1, 1} }
};

void simulate(std::vector<particle_structure>& particles, float dt_arg)
{

    size_t const N_substep = 10;
    float const dt = dt_arg / N_substep;
    for (size_t k_substep = 0; k_substep < N_substep; ++k_substep)
    {

        vec3 const g = { 0,0,-9.81f };
        size_t const N = particles.size();



        // **************************************** //
        // TO DO: Collision Handling
        // 
        // Handle the collision between the spheres and the cube faces.
        //
        //  Hints:
        //    - The cube is by default centered around zero, faces are between [-1,1]
        //      To compute the intersection between sphere and cube faces, you may store
        //      the normals, and face center positions of the cube in a vectors.
        //
        //    - Implement and check first the collision between the spheres and the cube
        //      before the collision between spheres
        //
        // **************************************** //
        for (size_t k = 0; k < N; k++)
        {
            particle_structure& particle = particles[k];

            for (size_t i = 0; i < 6; i++)
            {
                if (dot(particle.p - planes[i].origin, planes[i].n) <= particle.r)
                {
                    vec3 parra = -(dot(particle.v, planes[i].n) * planes[i].n);
                    vec3 ortho = -(particle.v -(dot(particle.v, planes[i].n) * planes[i].n));
                    particle.v = parra - ortho;
                }
            }
        }

        // Update velocity with gravity force and friction
        for (size_t k = 0; k < N; ++k)
        {
            particle_structure& particle = particles[k];
            vec3 const f = particle.m * g;

            particle.v = (1 - 0.9f * dt) * particle.v + dt * f / particle.m;
        }

        // Update position from velocity
        for (size_t k = 0; k < N; ++k)
        {
            particle_structure& particle = particles[k];
            particle.p = particle.p + dt * particle.v;
        }
    }

}
