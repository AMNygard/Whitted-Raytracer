//
//  Object.h
//  RAYTRACER
//
//  Created by Brian Summa on 11/17/15.
//
//

#ifndef __RAYTRACER__Object__
#define __RAYTRACER__Object__

#include "Angel.h"
#include "ObjMesh.h"
#include <assert.h>

#define EPSILON  1e-3

class Object{
public:
    
    std::string name;
    
    friend class Sphere;
    friend class Square;
    
    typedef struct{
        vec4 color;
        vec4 Kd;
        float Ks;
        float Kn;
        float Kt;
        float Ka;
        float Kr;
    } ShadingValues;
    
    typedef struct{
        //t in object space -> length of ray before intersection
        double t_o;
        //t in world
        double t_w;
        //p in object space
        vec4 P_o;
        //p in world space
        vec4 P_w;
        //Normal in object space
        vec4 N_o;
        //Normal in world space
        vec4 N_w;
        //The object hit
        Object *object;
    } IntersectionValues;
    
    
    Object(std::string name): name(name)  {};
    ~Object() {};
    
    Mesh mesh;
    ShadingValues shadingValues;
    
private:
    mat4 C;
    mat4 CInv;
    mat4 CInvStar;
    mat4 CInvTran;
    
public:
    
    void setShadingValues(ShadingValues _shadingValues){shadingValues = _shadingValues;}
    
    void setModelView(mat4 modelview){
        C = modelview;
        CInv = invert(modelview);
        mat4 CStar = modelview;
        CStar[0][3] = 0;
        CStar[1][3] = 0;
        CStar[2][3] = 0;
        CInvStar = invert(CStar);
        CInvTran = transpose(invert(modelview));
    }
    
    mat4 getModelView(){ return C; }
    
    virtual IntersectionValues intersect(vec4 p0, vec4 V)=0;
    
    
};

class Sphere : public Object{
public:
    
    Sphere(std::string name) : Object(name) { mesh.makeSubdivisionSphere(8); };
    
    /* ------------------------------------------------------------------------ */
    /* ------------------- Intersect Ray = p0+w + t*V_w  ---------------------- */
    virtual IntersectionValues intersect(vec4 p0_w, vec4 V_w){
        
        IntersectionValues result;
        result.object = this;
        
        vec4 p0_o = CInv*p0_w;
        vec4 V_o = normalize(CInvStar*V_w);
        
        result.t_o = sphereIntersect(p0_o, V_o);
        result.P_o = p0_o + result.t_o*V_o;
        
        result.t_w = result.t_o / length(CInvStar*V_w);
        result.P_w = p0_w + result.t_w * V_w;
        
        result.N_o = result.P_o;
        result.N_o.w = 0.0;
        
        result.N_w = CInvTran*result.N_o;
        result.N_w.w = 0.0;
        result.N_w = normalize(result.N_w);
        
        return result;
    }
    
    /* ------------------------------------------------------------------------ */
    /* ----- Ray = p0 + t*V  sphere at origin O and radius r    : Find t ------ */
    double sphereIntersect(vec4 p0, vec4 V, vec4 O=vec4(0.0, 0.0, 0.0, 1.0), double r=1.0){
        double a = 1;
        double b = dot(2*V, p0 - O);
        double c = pow(length(p0 - O), 2) - pow(r, 2);
        
        double t = -1*b / (2*a);
        double delta = pow(b, 2) - 4*a*c;
        
        if (nearlyEqual(delta, 0.0, EPSILON)){
            return t;
        }
        else if (delta < 0.0) {
            return std::numeric_limits<double>::infinity();
        }
        else {
            delta = pow(delta, 0.5) / (2*a);
            double tSmall = t - delta;
            double tBig = t + delta;
            
            if ((tBig < 0 || nearlyEqual(tBig, 0, EPSILON)) && !(tSmall < 0 || nearlyEqual(tSmall, 0, EPSILON))) {
                return tSmall;
            } else if (!(tBig < 0 || nearlyEqual(tBig, 0, EPSILON)) && (tSmall < 0 || nearlyEqual(tSmall, 0, EPSILON))) {
                return tBig;
            } else if ((tBig < 0 || nearlyEqual(tBig, 0, EPSILON)) && (tSmall < 0 || nearlyEqual(tSmall, 0, EPSILON))){
                return std::numeric_limits<double>::infinity();
            } else {
                if (tBig<tSmall){
                    return tBig;
                } else {
                    return tSmall;
                }
            }
        }
    }
    
    
};

class Square : public Object{
public:
    Square(std::string name) : Object(name) {
        
        mesh.vertices.resize(6);
        mesh.uvs.resize(6);
        mesh.normals.resize(6);
        
        mesh.vertices[0]=vec4(-1.0, -1.0, 0.0, 1.0);
        mesh.uvs[0] = vec2(0.0,0.0);
        mesh.vertices[1]=vec4(1.0, 1.0, 0.0, 1.0);
        mesh.uvs[1] = vec2(1.0,1.0);
        mesh.vertices[2]=vec4(1.0, -1.0, 0.0, 1.0);
        mesh.uvs[2] = vec2(1.0,0.0);
        
        mesh.vertices[3]=vec4(-1.0, -1.0, 0.0, 1.0);
        mesh.uvs[3] = vec2(0.0,0.0);
        mesh.vertices[4]=vec4(1.0, 1.0, 0.0, 1.0);
        mesh.uvs[4] = vec2(1.0,1.0);
        mesh.vertices[5]=vec4(-1.0, 1.0, 0.0, 1.0);
        mesh.uvs[5] = vec2(0.0,1.0);
        
        mesh.normals[0]= vec3(0, 0, 1.0);
        mesh.normals[1]= vec3(0, 0, 1.0);
        mesh.normals[2]= vec3(0, 0, 1.0);
        mesh.normals[3]= vec3(0, 0, 1.0);
        mesh.normals[4]= vec3(0, 0, 1.0);
        mesh.normals[5]= vec3(0, 0, 1.0);
        
    };
    
    /* ------------------------------------------------------------------------ */
    /* -------------------- Ray = p0 + t*V  with square ----------------------- */
    virtual IntersectionValues intersect(vec4 p0_w, vec4 V_w){
        IntersectionValues result;
        result.object = this;
        
        vec4 p0_o = CInv*p0_w;
        vec4 V_o = normalize(CInvStar*V_w);
        
        result.t_o = squareIntersect(p0_o, V_o);
        result.P_o = p0_o + result.t_o*V_o;
        
        result.t_w = result.t_o / length(CInvStar*V_w);
        result.P_w = p0_w + result.t_w * V_w;
        
        result.N_o = vec4(0.0, 0.0, 1.0, 0.0);
        result.N_w = CInvTran*result.N_o;
        result.N_w.w = 0.0;
        result.N_w = normalize(result.N_w);
        
        return result;
    }
    
    /* ------------------------------------------------------------------------ */
    /* ----------------- Ray = p0 + t*V  with square: Find t ------------------ */
    double squareIntersect(vec4 p0, vec4 V, vec4 N=vec4(0.0,0.0,1.0,0.0), vec4 S=vec4(0.0, 0.0,0.0,1.0)){
        
        //dear god, this is still translating my squares
        //but the shadow intersections seem to be working fine?
        //look for a difference there
        
        if (!nearlyEqual(dot(N, V), 0.0, EPSILON)) {
            double t = dot(N, S-p0)/ dot(N, V);
            
            if (t < 0 || nearlyEqual(t, 0.0, EPSILON)) {
                return std::numeric_limits<double>::infinity();
            }
            
            vec4 P = p0 + t*V;
            
            if ((P.x > 1.0 || P.x < -1.0) || (P.y > 1.0 || P.y < -1.0)){
                return std::numeric_limits< double >::infinity();
            }
            else {
                return t;
            }
        }
        else {
            return std::numeric_limits<double>::infinity();
        }
    }
    
};
#endif /* defined(__RAYTRACER__Objects__) */
