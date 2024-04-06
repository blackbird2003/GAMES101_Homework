//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

#define eps 1e-6

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
    //recursive until get a point
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    /*
    shade(p, wo)
        sampleLight(inter , pdf_light)
        Get x, ws, NN, emit from inter
        Shoot a ray from p to x
        If the ray is not blocked in the middle
            L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws,
            NN) / |x-p|^2 / pdf_light
        
        
        L_indir = 0.0
        //Test Russian Roulette with probability RussianRoulette
        wi = sample(wo, N)
        Trace a ray r(p, wi)
        If ray r hit a non-emitting object at q
            L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N)
            / pdf(wo, wi, N) / RussianRoulette
        
        Return L_dir + L_indir
    */

    Vector3f L_dir(0, 0, 0), L_indir(0, 0, 0);

    //ray wo is to p, now find p and see if already hit light
    Ray wo = ray;
    Intersection p_inter = this->intersect(wo);
    //if hit nothing
    if (!p_inter.happened) return L_dir;
    //if hit light source
    if (p_inter.m->hasEmission()) return p_inter.m->getEmission();

    //otherwise, it hit a object, continue

    //sampleLight(inter , pdf_light)
    //uniformly sample x from all LIGHTS and get its pdf
    Intersection x_inter; float x_pdf;
    sampleLight(x_inter, x_pdf);

    //Get x, ws, Nx, emit from inter 
    //ws is from p to x, Np is at p, Nx is at x
    Vector3f p = p_inter.coords;
    Vector3f x = x_inter.coords;
    Vector3f Np = p_inter.normal;
    Vector3f Nx = x_inter.normal;
    Vector3f emit = x_inter.emit;    

    //Shoot a ray (ws) from p to x 
    Vector3f ws_dir = (x - p).normalized();
    Ray ws(p, ws_dir);
    Intersection ws_inter = this->intersect(ws);

    // If the ray is NOT blocked in the middle
    //         L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws,
    //         NN) / |x-p|^2 / pdf_light
    // Else L_dir = 0.0

    //calc length of p - x and ws_inter to see if it is blocked
    float px_dis = (x - p).norm(), ws_dis = ws_inter.distance;
    if (px_dis - ws_dis > eps) {
        L_dir = emit 
        * p_inter.m->eval(wo.direction, ws.direction, Np)
        * dotProduct(ws.direction, Np)      //all vectors were nomorlized
        * dotProduct(-ws.direction, Nx)     //so dot product is cosine
        / (ws_dis * ws_dis)
        / x_pdf;
    } // else L_dir = 0; no need


    //Now calculate L_indir
    //Test Russian Roulette with probability RussianRoulette
    float P_rand = get_random_float();
    if (P_rand < RussianRoulette) {
        //wi = sample(wo, N)
        //wi is from p to q
        Vector3f wi_dir = p_inter.m->sample(wo.direction, Np).normalized();
        Ray wi(p_inter.coords, wi_dir);
        // Trace a ray r(p, wi)
        // If ray r hit a non-emitting object at q
        //     L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N)
        //     / pdf(wo, wi, N) / RussianRoulette
        Intersection wi_inter = this->intersect(wi);
        if (wi_inter.happened && !(wi_inter.m->hasEmission())) {
            L_indir = castRay(wi, depth + 1)
            * p_inter.m->eval(wo.direction, wi.direction, Np)
            * dotProduct(wi.direction, Np)
            / p_inter.m->pdf(wo.direction, wi.direction, Np)
            / RussianRoulette;
        }
    }
    return L_dir + L_indir;
}