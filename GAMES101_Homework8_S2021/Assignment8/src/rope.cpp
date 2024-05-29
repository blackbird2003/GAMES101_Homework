#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.


        // Def of rope:
        // vector<Mass *> masses;
        // vector<Spring *> springs;

        /*
          Mass(Vector2D position, float mass, bool pinned)
      : start_position(position), position(position), last_position(position),
        mass(mass), pinned(pinned) {}
        Spring(Mass *a, Mass *b, float k)
      : m1(a), m2(b), k(k), rest_length((a->position - b->position).norm()) {}
        */
        for (int i = 0; i < num_nodes; i++) {
            Vector2D pos = start + (end - start) * (1.0 * i / (num_nodes - 1));
            masses.emplace_back(new Mass(pos, node_mass, false));
        }

        for (int i = 0; i < num_nodes - 1; i++) {
            springs.emplace_back(new Spring(masses[i], masses[i + 1], k));
        } 

        // Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto len = (s->m1->position - s->m2->position).norm();
            s->m1->forces += - s->k * (s->m1->position - s->m2->position) / len * (len - s->rest_length);
            s->m2->forces += - s->k * (s->m2->position - s->m1->position) / len * (len - s->rest_length);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                auto a = m->forces / m->mass + gravity;
                float kd = 0.005; a += - kd * m->velocity / m->mass;  // TODO (Part 2): Add global damping
                auto v_t = m->velocity;
                m->velocity += a * delta_t;
                //m->position += v_t * delta_t;  //Explicit Method 不收敛
                m->position += m->velocity * delta_t; // Semi-implicit method
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            auto len = (s->m1->position - s->m2->position).norm();
            s->m1->forces += -s->k * (s->m1->position - s->m2->position) / len * (len - s->rest_length);
            s->m2->forces += -s->k * (s->m2->position - s->m1->position) / len * (len - s->rest_length);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                // TODO (Part 4): Add global Verlet damping
                auto a = m->forces / m->mass + gravity;
                float damping_factor = 0.00000000005;
                m->position = temp_position + (1 - damping_factor) * (temp_position - m->last_position) + a * delta_t * delta_t; 
                m->last_position = temp_position;
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
