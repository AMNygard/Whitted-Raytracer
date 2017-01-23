#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "Angel.h"
#include "SourcePath.h"
#include "Trackball.h"
#include "ObjMesh.h"
#include "Object.h"

#include <algorithm>
#include "pngenc.h"

using namespace Angel;

typedef vec4  color4;
typedef vec4  point4;

//Scene variables
std::vector < Object * > sceneObjects;
point4 lightPosition;
color4 lightColor;
point4 cameraPosition;

//Recursion depth for raytracer
int maxDepth = 4;

namespace GLState {
  int window_width, window_height;
  
  bool render_line;
  
  std::vector < GLuint > objectVao;
  std::vector < GLuint > objectBuffer;

  GLuint vPosition, vNormal, vTexCoord;
  
  GLuint program;
  
  // Model-view and projection matrices uniform location
  GLuint  ModelView, ModelViewLight, NormalMatrix, Projection;
  
  //==========Trackball Variables==========
  static float curquat[4],lastquat[4];
  /* current transformation matrix */
  static float curmat[4][4];
  mat4 curmat_a;
  /* actual operation  */
  static int scaling;
  static int moving;
  static int panning;
  /* starting "moving" coordinates */
  static int beginx, beginy;
  /* ortho */
  float ortho_x, ortho_y;
  /* current scale factor */
  static float scalefactor;
  
  mat4  projection;
  mat4 sceneModelView;
  
  color4 light_ambient;
  color4 light_diffuse;
  color4 light_specular;
  
};

/* ------------------------------------------------------- */
/* -- PNG receptor class for use with pngdecode library -- */
class rayTraceReceptor : public cmps3120::png_receptor
{
private:
  const unsigned char *buffer;
  unsigned int width;
  unsigned int height;
  int channels;
  
public:
  rayTraceReceptor(const unsigned char *use_buffer,
    unsigned int width,
    unsigned int height,
    int channels){
    this->buffer = use_buffer;
    this->width = width;
    this->height = height;
    this->channels = channels;
  }
  cmps3120::png_header get_header(){
    cmps3120::png_header header;
    header.width = width;
    header.height = height;
    header.bit_depth = 8;
    switch (channels)
    {
    case 1:
        header.color_type = cmps3120::PNG_GRAYSCALE;break;
    case 2:
        header.color_type = cmps3120::PNG_GRAYSCALE_ALPHA;break;
    case 3:
        header.color_type = cmps3120::PNG_RGB;break;
    default:
        header.color_type = cmps3120::PNG_RGBA;break;
    }
    return header;
  }
  cmps3120::png_pixel get_pixel(unsigned int x, unsigned int y, unsigned int level){
    cmps3120::png_pixel pixel;
    unsigned int idx = y*width+x;
    /* pngdecode wants 16-bit color values */
    pixel.r = buffer[4*idx]*257;
    pixel.g = buffer[4*idx+1]*257;
    pixel.b = buffer[4*idx+2]*257;
    pixel.a = buffer[4*idx+3]*257;
    return pixel;
  }
};

/* -------------------------------------------------------------------------- */
/* ----------------------  Write Image to Disk  ----------------------------- */
bool write_image(const char* filename, const unsigned char *Src,
                 int Width, int Height, int channels){
  cmps3120::png_encoder the_encoder;
  cmps3120::png_error result;
  rayTraceReceptor image(Src,Width,Height,channels);
  the_encoder.set_receptor(&image);
  result = the_encoder.write_file(filename);
  if (result == cmps3120::PNG_DONE)
    std::cerr << "finished writing "<<filename<<"."<<std::endl;
  else
    std::cerr << "write to "<<filename<<" returned error code "<<result<<"."<<std::endl;
  return result==cmps3120::PNG_DONE;
}


/* -------------------------------------------------------------------------- */
/* -------- Given OpenGL matrices find ray in world coordinates of ---------- */
/* -------- window position x,y --------------------------------------------- */
std::vector < vec4 > findRay(GLdouble x, GLdouble y){
  
  y = GLState::window_height-y;
  
  int viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);

    GLdouble modelViewMatrix[16];
    GLdouble projectionMatrix[16];
    for(unsigned int i=0; i < 4; i++){
      for(unsigned int j=0; j < 4; j++){
        modelViewMatrix[j*4+i]  =  GLState::sceneModelView[i][j];
        projectionMatrix[j*4+i] =  GLState::projection[i][j];
      }
    }

  
  GLdouble nearPlaneLocation[3];
  _gluUnProject(x, y, 0.0, modelViewMatrix, projectionMatrix,
               viewport, &nearPlaneLocation[0], &nearPlaneLocation[1],
               &nearPlaneLocation[2]);
  
  GLdouble farPlaneLocation[3];
  _gluUnProject(x, y, 1.0, modelViewMatrix, projectionMatrix,
               viewport, &farPlaneLocation[0], &farPlaneLocation[1],
               &farPlaneLocation[2]);
  
  
  vec4 ray_origin = vec4(nearPlaneLocation[0],
                         nearPlaneLocation[1],
                         nearPlaneLocation[2], 1.0);
  vec3 temp = vec3(farPlaneLocation[0]-nearPlaneLocation[0],
                   farPlaneLocation[1]-nearPlaneLocation[1],
                   farPlaneLocation[2]-nearPlaneLocation[2]);
  temp = normalize(temp);
  vec4 ray_dir = vec4(temp.x, temp.y, temp.z, 0.0);
  
  std::vector < vec4 > result(2);
  result[0] = ray_origin;
  result[1] = ray_dir;
  
  return result;
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
bool intersectionSort(Object::IntersectionValues i, Object::IntersectionValues j){
  return (i.t_w < j.t_w);
}

/* -------------------------------------------------------------------------- */
bool shadowFeeler(vec4 p0, Object* object){
    vec4 dir = normalize(lightPosition - p0);
    float tLight = (lightPosition.y - p0.y)/dir.y;
    vec4 P = p0 + EPSILON*dir;
    
    
    Object * currObj;
    Object::IntersectionValues currIntersect;
    Object::IntersectionValues visibleIntersect = sceneObjects.at(0)->intersect(p0, dir);
    for (int i = 0; i < sceneObjects.size(); i++){
        currObj = sceneObjects.at(i);
        
        if (currObj != object){
            currIntersect = currObj->intersect(P, dir);
            
            if (visibleIntersect.t_w > currIntersect.t_w){
                visibleIntersect = currIntersect;
            }
        }
    }
    

    if (visibleIntersect.t_w != std::numeric_limits<double>::infinity() && visibleIntersect.t_w < tLight && visibleIntersect.t_w > 0){
        return true;
    }
    
    return false;
}

/* -------------------------------------------------------------------------- */
/* ----------  cast Ray = p0 + t*dir and intersect with scene       --------- */
/* -------------------------------------------------------------------------- */

vec4 fresnelDir(vec4 N, vec4 I, float n) {
    vec4 newDir;
    float sqArg = 1 - n*n * (1 - dot(N, I)*dot(N,I));
    
    if (nearlyEqual(sqArg, 0, EPSILON) || sqArg < 0) {
        newDir = vec4(0.0, 0.0,0.0,-1.0);//normalize(((-n)*dot(N, I))*N + n*I);
    } else {
        newDir = ((-n)*dot(N, I) - pow(sqArg, 0.5)) * N + n*I;
        newDir.w = 0.0;
        newDir = normalize(newDir);
    }
    return newDir;
}

float reflectance(vec4 N, vec4 I, float n1, float n2) {
    float rTot;
    
    float n = n1 / n2;
    
    float cosi = -dot(N, I);
    float sin2t = n*n*(1 - cosi*cosi);
    float cost = pow(1 - sin2t, 0.5);
    
    float rPerp = pow((n1*cosi - n2*cost)/(n1*cosi + n2*cost), 2);
    float rPara = pow((n2*cosi - n1*cost)/(n2*cosi + n1*cost), 2);
    
    rTot = (rPerp+rPara)/2;
    
    return rTot;
}

vec4 castRay(vec4 p0, vec4 dir, Object *lastHitObject, int depth){
    vec4 color = vec4(0.0,0.0,0.0,0.0);
    
    if(depth > maxDepth){ return color; }
    
    Object * currObj;
    Object::IntersectionValues currIntersect;
    Object::IntersectionValues visibleIntersect = sceneObjects.at(0)->intersect(p0, dir);
    for (int i = 0; i < sceneObjects.size(); i++){
        currObj = sceneObjects.at(i);
        
        if (currObj != lastHitObject || currObj->shadingValues.Kt != 0){
            currIntersect = currObj->intersect(p0, dir);
            
            if (visibleIntersect.t_w > currIntersect.t_w){
                visibleIntersect = currIntersect;
            }
        }
    }
    
    if (visibleIntersect.t_w == std::numeric_limits<double>::infinity()) {
        //todo: check for parallel to light source,
        //not necessary with the current scene but good to do with other scenes
    }
    else {
        color.w = 1.0;
        
        float Ka = visibleIntersect.object->shadingValues.Ka;
        vec4 Kd = visibleIntersect.object->shadingValues.Kd;
        float Ks = visibleIntersect.object->shadingValues.Ks;
        float Kn = visibleIntersect.object->shadingValues.Kn;
        float Kt = visibleIntersect.object->shadingValues.Kt;
        
        vec4 objColor = visibleIntersect.object->shadingValues.color; objColor.w = 1.0;
        vec4 ambient = vec4(0.0, 0.0,0.0,0.0);
        vec4 diffuse = vec4(0.0, 0.0,0.0,0.0);
        vec4 specular = vec4(0.0,0.0,0.0,0.0);
        vec4 transmitted = vec4(0.0,0.0,0.0,0.0);
        
        
        vec4 L;
        if(lightPosition.w == 0.0){
            L = -lightPosition;
        }else{
            L = normalize(lightPosition - visibleIntersect.P_w ); L.w = 0;
        }
        vec4 N = visibleIntersect.N_w;
        
        
        if (Ka > 0) {
            ambient = Ka*objColor*lightColor;
        }
        
        if (Kd > 0) {
            diffuse = Kd * lightColor * objColor * dot(N, L);
        }
        
        if (Ks > 0) {
            vec4 materialSpecular = vec4(Ks, Ks, Ks, 1);
            vec4 lightSpecular = vec4(lightColor.x, lightColor.y, lightColor.z, 1);
            
            vec4 R = normalize(-reflect(L, N));
            vec4 V = normalize(cameraPosition - visibleIntersect.P_w); V.w = 0.0;
            if (depth == 0){
                V = normalize(p0 - visibleIntersect.P_w);
            }
            
            vec4 refDir = normalize(reflect(dir, N));
            vec4 specCol = Ks*castRay(visibleIntersect.P_w, refDir, visibleIntersect.object, depth + 1);
            
            
            float specProd = 0.0;
            float S = dot(V, R);
            if (S < 0.0) {
                S = 0.0;
            }
            
            if (!nearlyEqual(S, 0.0, EPSILON) || !nearlyEqual(Kn, 0.0, EPSILON)) { specProd = pow(S, Kn);}
            
            specular = specCol + specProd * lightSpecular * materialSpecular;// * specProd;
            //TODO: separate this into two terms: the reflection and the highlight
            
        }
        
        
        /*END SPECULAR CODE*/
        
        if (Kt > 0) {
            
            float n1 = visibleIntersect.object->shadingValues.Kr;
            float n2 = 1.0;
            
            vec4 N = visibleIntersect.N_w;
            vec4 I = dir;
            
            if (visibleIntersect.object != lastHitObject){ //leaving the object, second n should be vacuum
                n1 = 1;
                n2 = visibleIntersect.object->shadingValues.Kr;
            } else { //entering the object, use internal normal for purposes of ray recursion
                N = -1*N;
            }
            
            float refl = reflectance(N, I, n1, n2);
            float refr = 1.0 - refl;
            
            vec4 refrDir = fresnelDir(N, I, n1/n2);
            vec4 reflDir = normalize(reflect(I, N));
            
            vec4 refrCol, reflCol;
            
            if (refrDir.w != -1.0) {
                refrCol = Kt*refr*castRay(visibleIntersect.P_w+EPSILON*refrDir, refrDir, visibleIntersect.object, depth+1);
            }
            
            reflCol = refl*castRay(visibleIntersect.P_w+EPSILON*reflDir, reflDir,visibleIntersect.object, depth+1);
            
            transmitted = reflCol + refrCol;
            
            if (visibleIntersect.object == lastHitObject) {
                N = -1*N;
            }
            
        }
        
        
        /*END TRANSMITTIVE CODE*/
        
        bool inShadow = shadowFeeler(visibleIntersect.P_w, visibleIntersect.object);
        if (!inShadow) {
            
            if (dot(N, L) > 0) {
                color = ambient + diffuse;// + specular;// + transmitted;
            }
            
            if (Ks > 0) {
                color = color + specular;
            }
            
            if (Kt > 0){ //pretty sure the transmitted color should be visible even if dot(N, L) <= 0
                color = color + transmitted;
            }
            
        }
        color.w = 1;
        
        //if i don't manually set the w value to 1, it doesn't get properly calculated
        //fixing this is currently low priority, the behavior of dot(N, L) needs to be
        //fixed first
        
    }
    
    
    
    
    if (color.x > 1) {
        color.x = 1;
    }
    if (color.y > 1) {
        color.y = 1;
    }
    if (color.z > 1) {
        color.z = 1;
    }
    
    return color;
}
/* -------------------------------------------------------------------------- */
/* ------------  Ray trace our scene.  Output color to image and    --------- */
/* -----------   Output color to image and save to disk             --------- */
void rayTrace(){
  
  unsigned char *buffer = new unsigned char[GLState::window_width*GLState::window_height*4];
  
  for(unsigned int i=0; i < GLState::window_width; i++){
    for(unsigned int j=0; j < GLState::window_height; j++){
      
      int idx = j*GLState::window_width+i;
      std::vector < vec4 > ray_o_dir = findRay(i,j);
      vec4 color = castRay(ray_o_dir[0], ray_o_dir[1], NULL, 0);
      buffer[4*idx]   = color.x*255;
      buffer[4*idx+1] = color.y*255;
      buffer[4*idx+2] = color.z*255;
      buffer[4*idx+3] = color.w*255;
    }
  }
  
  write_image("output.png", buffer, GLState::window_width, GLState::window_height, 4);
  
  delete[] buffer;
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
static void error_callback(int error, const char* description)
{
  fprintf(stderr, "Error: %s\n", description);
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  if (key == GLFW_KEY_R && action == GLFW_PRESS)
    rayTrace();
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
static void mouseClick(GLFWwindow* window, int button, int action, int mods){
  
  if (GLFW_RELEASE == action){
    GLState::moving=GLState::scaling=GLState::panning=false;
    return;
  }
  
  if( mods & GLFW_MOD_SHIFT){
    GLState::scaling=true;
  }else if( mods & GLFW_MOD_ALT ){
    GLState::panning=true;
  }else{
    GLState::moving=true;
    trackball(GLState::lastquat, 0, 0, 0, 0);
  }
  
  double xpos, ypos;
  glfwGetCursorPos(window, &xpos, &ypos);
  GLState::beginx = xpos; GLState::beginy = ypos;
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
void mouseMove(GLFWwindow* window, double x, double y){
  
  int W, H;
  glfwGetFramebufferSize(window, &W, &H);

  
  float dx=(x-GLState::beginx)/(float)W;
  float dy=(GLState::beginy-y)/(float)H;
  
  if (GLState::panning)
    {
    GLState::ortho_x  +=dx;
    GLState::ortho_y  +=dy;
    
    GLState::beginx = x; GLState::beginy = y;
    return;
    }
  else if (GLState::scaling)
    {
    GLState::scalefactor *= (1.0f+dx);
    
    GLState::beginx = x;GLState::beginy = y;
    return;
    }
  else if (GLState::moving)
    {
    trackball(GLState::lastquat,
              (2.0f * GLState::beginx - W) / W,
              (H - 2.0f * GLState::beginy) / H,
              (2.0f * x - W) / W,
              (H - 2.0f * y) / H
              );
    
    add_quats(GLState::lastquat, GLState::curquat, GLState::curquat);
    build_rotmatrix(GLState::curmat, GLState::curquat);
    
    GLState::beginx = x;GLState::beginy = y;
    return;
    }
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
void initScene(){
  cameraPosition = point4( 0.0, 0.0, 6.0, 1.0 );
  lightPosition = point4( 0.0, 0.0, 0.0, 1.0 );
  lightColor = color4( 1.0, 0.0, 0.0, 1.0);
  
//  {
//  sceneObjects.push_back(new Sphere("Sphere"));
//  Object::ShadingValues _shadingValues;
//  _shadingValues.color = vec4(1.0,1.0,1.0,1.0);
//  _shadingValues.Ka = 0.0;
//  _shadingValues.Kd = 1.0;
//  _shadingValues.Ks = 0.0;
//  _shadingValues.Kn = 0.0;
//  _shadingValues.Kt = 0.0;
//  _shadingValues.Kr = 0.0;
//  sceneObjects[sceneObjects.size()-1]->setShadingValues(_shadingValues);
//  sceneObjects[sceneObjects.size()-1]->setModelView(Scale(0.5, 0.5, 0.5));
//  }

  
    { //Back Wall
        sceneObjects.push_back(new Square("Back Wall"));
        Object::ShadingValues _shadingValues;
        _shadingValues.color = vec4(0.0,0.0,0.5,1.0);
        _shadingValues.Ka = 0.0;
        _shadingValues.Kd = 0.2;
        _shadingValues.Ks = 0.8;
        _shadingValues.Kn = 16.0;
        _shadingValues.Kt = 0.0;
        _shadingValues.Kr = 0.0;
        sceneObjects[sceneObjects.size()-1]->setShadingValues(_shadingValues);
        sceneObjects[sceneObjects.size()-1]->setModelView(Translate(0.0, 0.0, -2.0)*Scale(2.0,2.0,1.0));
    }
    
    { //Left Wall
        sceneObjects.push_back(new Square("Left Wall"));
        Object::ShadingValues _shadingValues;
        _shadingValues.color = vec4(0.0,0.0,1.0,1.0);
        _shadingValues.Ka = 0.0;
        _shadingValues.Kd = 1.0;
        _shadingValues.Ks = 0.0;
        _shadingValues.Kn = 16.0;
        _shadingValues.Kt = 0.0;
        _shadingValues.Kr = 0.0;
        sceneObjects[sceneObjects.size()-1]->setShadingValues(_shadingValues);
        sceneObjects[sceneObjects.size()-1]->setModelView(RotateY(90)*Translate(0.0, 0.0, -2.0)*Scale(2.0,2.0,1.0));
    }
    
    { //Right Wall
        sceneObjects.push_back(new Square("Right Wall"));
        Object::ShadingValues _shadingValues;
        _shadingValues.color = vec4(0.5,0.0,0.5,1.0);
        _shadingValues.Ka = 0.0;
        _shadingValues.Kd = 1.0;
        _shadingValues.Ks = 0.0;
        _shadingValues.Kn = 16.0;
        _shadingValues.Kt = 0.0;
        _shadingValues.Kr = 0.0;
        sceneObjects[sceneObjects.size()-1]->setShadingValues(_shadingValues);
        sceneObjects[sceneObjects.size()-1]->setModelView(RotateY(-90)*Translate(0.0, 0.0, -2.0)*Scale(2.0, 2.0, 1.0 ));
    }
    
    { //Floor
        sceneObjects.push_back(new Square("Floor"));
        Object::ShadingValues _shadingValues;
        _shadingValues.color = vec4(0.0,1.0,1.0,1.0);
        _shadingValues.Ka = 0.0;
        _shadingValues.Kd = 1.0;
        _shadingValues.Ks = 0.0;
        _shadingValues.Kn = 16.0;
        _shadingValues.Kt = 0.0;
        _shadingValues.Kr = 0.0;
        sceneObjects[sceneObjects.size()-1]->setShadingValues(_shadingValues);
        sceneObjects[sceneObjects.size()-1]->setModelView(RotateX(-90)*Translate(0.0, 0.0, -2.0)*Scale(2.0, 2.0, 1.0));
    }
    
    { //Ceiling
        sceneObjects.push_back(new Square("Ceiling"));
        Object::ShadingValues _shadingValues;
        _shadingValues.color = vec4(1.0,1.0,1.0,1.0);
        _shadingValues.Ka = 0.0;
        _shadingValues.Kd = 1.0;
        _shadingValues.Ks = 0.0;
        _shadingValues.Kn = 16.0;
        _shadingValues.Kt = 0.0;
        _shadingValues.Kr = 0.0;
        sceneObjects[sceneObjects.size()-1]->setShadingValues(_shadingValues);
        sceneObjects[sceneObjects.size()-1]->setModelView(RotateX(90)*Translate(0.0, 0.0, -2.0)*Scale(2.0, 2.0, 1.0));
    }
    
    { //Front Wall
        sceneObjects.push_back(new Square("Front Wall"));
        Object::ShadingValues _shadingValues;
        _shadingValues.color = vec4(0.0,1.0,0.0,1.0);
        _shadingValues.Ka = 0.0;
        _shadingValues.Kd = 0.2;
        _shadingValues.Ks = 0.8;
        _shadingValues.Kn = 16.0;
        _shadingValues.Kt = 0.0;
        _shadingValues.Kr = 0.0;
        sceneObjects[sceneObjects.size()-1]->setShadingValues(_shadingValues);
        sceneObjects[sceneObjects.size()-1]->setModelView(RotateY(180)*Translate(0.0, 0.0, -2.0)*Scale(2.0, 2.0, 1.0));
    }
    
    
    {
        sceneObjects.push_back(new Sphere("Glass sphere"));
        Object::ShadingValues _shadingValues;
        _shadingValues.color = vec4(0.0,0.0,1.0,1.0);
        _shadingValues.Ka = 0.0;
        _shadingValues.Kd = 0.0;
        _shadingValues.Ks = 0.0;
        _shadingValues.Kn = 16.0;
        _shadingValues.Kt = 1.0;
        _shadingValues.Kr = 1.4;
        sceneObjects[sceneObjects.size()-1]->setShadingValues(_shadingValues);
        sceneObjects[sceneObjects.size()-1]->setModelView(Translate(1.1, -1.5, 0.6)*Scale(0.5, 0.5, 0.5));
    }
    
    {
        sceneObjects.push_back(new Sphere("Mirrored Sphere"));
        Object::ShadingValues _shadingValues;
        _shadingValues.color = vec4(1.0,1.0,1.0,1.0);
        _shadingValues.Ka = 0.0;
        _shadingValues.Kd = 0.0;
        _shadingValues.Ks = 1.0;
        _shadingValues.Kn = 16.0;
        _shadingValues.Kt = 0.0;
        _shadingValues.Kr = 0.0;
        sceneObjects[sceneObjects.size()-1]->setShadingValues(_shadingValues);
        sceneObjects[sceneObjects.size()-1]->setModelView(Translate(-1.1, -1.5, -0.6)*Scale(0.5, 0.5, 0.5));
    }
    
//    {
//        sceneObjects.push_back(new Sphere("Diffuse Sphere"));
//        Object::ShadingValues _shadingValues;
//        _shadingValues.color = vec4(0.0,1.0,0.0,1.0);
//        _shadingValues.Ka = 0.0;
//        _shadingValues.Kd = 0.7;
//        _shadingValues.Ks = 0.3;
//        _shadingValues.Kn = 16.0;
//        _shadingValues.Kt = 0.0;
//        _shadingValues.Kr = 1.4;
//        sceneObjects[sceneObjects.size()-1]->setShadingValues(_shadingValues);
//        sceneObjects[sceneObjects.size()-1]->setModelView(Scale(0.75, 0.75, 0.75));
//    }
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
void initGL(){
  
  GLState::light_ambient  = vec4(lightColor.x, lightColor.y, lightColor.z, 1.0 );
  GLState::light_diffuse  = vec4(lightColor.x, lightColor.y, lightColor.z, 1.0 );
  GLState::light_specular = vec4(lightColor.x, lightColor.y, lightColor.z, 1.0 );

  
  std::string vshader = source_path + "/shaders/vshader.glsl";
  std::string fshader = source_path + "/shaders/fshader.glsl";
  
  GLchar* vertex_shader_source = readShaderSource(vshader.c_str());
  GLchar* fragment_shader_source = readShaderSource(fshader.c_str());

  GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex_shader, 1, (const GLchar**) &vertex_shader_source, NULL);
  glCompileShader(vertex_shader);
  check_shader_compilation(vshader, vertex_shader);
  
  GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment_shader, 1, (const GLchar**) &fragment_shader_source, NULL);
  glCompileShader(fragment_shader);
  check_shader_compilation(fshader, fragment_shader);
  
  GLState::program = glCreateProgram();
  glAttachShader(GLState::program, vertex_shader);
  glAttachShader(GLState::program, fragment_shader);
  
  glLinkProgram(GLState::program);
  check_program_link(GLState::program);
  
  glUseProgram(GLState::program);
  
  glBindFragDataLocation(GLState::program, 0, "fragColor");

  // set up vertex arrays
  GLState::vPosition = glGetAttribLocation( GLState::program, "vPosition" );
  GLState::vNormal = glGetAttribLocation( GLState::program, "vNormal" );
  
  // Retrieve transformation uniform variable locations
  GLState::ModelView = glGetUniformLocation( GLState::program, "ModelView" );
  GLState::NormalMatrix = glGetUniformLocation( GLState::program, "NormalMatrix" );
  GLState::ModelViewLight = glGetUniformLocation( GLState::program, "ModelViewLight" );
  GLState::Projection = glGetUniformLocation( GLState::program, "Projection" );
  
  GLState::objectVao.resize(sceneObjects.size());
  glGenVertexArrays( sceneObjects.size(), &GLState::objectVao[0] );
  
  GLState::objectBuffer.resize(sceneObjects.size());
  glGenBuffers( sceneObjects.size(), &GLState::objectBuffer[0] );
  
  for(unsigned int i=0; i < sceneObjects.size(); i++){
    glBindVertexArray( GLState::objectVao[i] );
    glBindBuffer( GL_ARRAY_BUFFER, GLState::objectBuffer[i] );
    size_t vertices_bytes = sceneObjects[i]->mesh.vertices.size()*sizeof(vec4);
    size_t normals_bytes  =sceneObjects[i]->mesh.normals.size()*sizeof(vec3);
    
    glBufferData( GL_ARRAY_BUFFER, vertices_bytes + normals_bytes, NULL, GL_STATIC_DRAW );
    size_t offset = 0;
    glBufferSubData( GL_ARRAY_BUFFER, offset, vertices_bytes, &sceneObjects[i]->mesh.vertices[0] );
    offset += vertices_bytes;
    glBufferSubData( GL_ARRAY_BUFFER, offset, normals_bytes,  &sceneObjects[i]->mesh.normals[0] );
    
    glEnableVertexAttribArray( GLState::vNormal );
    glEnableVertexAttribArray( GLState::vPosition );

    glVertexAttribPointer( GLState::vPosition, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0) );
    glVertexAttribPointer( GLState::vNormal, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(vertices_bytes));

  }
  
  

  glEnable( GL_DEPTH_TEST );
  glShadeModel(GL_SMOOTH);

  glClearColor( 0.8, 0.8, 1.0, 1.0 );
  
  //Quaternion trackball variables, you can ignore
  GLState::scaling  = 0;
  GLState::moving   = 0;
  GLState::panning  = 0;
  GLState::beginx   = 0;
  GLState::beginy   = 0;
  
  matident(GLState::curmat);
  trackball(GLState::curquat , 0.0f, 0.0f, 0.0f, 0.0f);
  trackball(GLState::lastquat, 0.0f, 0.0f, 0.0f, 0.0f);
  add_quats(GLState::lastquat, GLState::curquat, GLState::curquat);
  build_rotmatrix(GLState::curmat, GLState::curquat);
  
  GLState::scalefactor = 1.0;
  GLState::render_line = false;
  
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
void drawObject(Object * object, GLuint vao, GLuint buffer){
  
  color4 material_ambient(object->shadingValues.color.x*object->shadingValues.Ka,
                          object->shadingValues.color.y*object->shadingValues.Ka,
                          object->shadingValues.color.z*object->shadingValues.Ka, 1.0 );
  color4 material_diffuse(object->shadingValues.color.x,
                          object->shadingValues.color.y,
                          object->shadingValues.color.z, 1.0 );
  color4 material_specular(object->shadingValues.Ks,
                           object->shadingValues.Ks,
                           object->shadingValues.Ks, 1.0 );
  float  material_shininess = object->shadingValues.Kn;
  
  color4 ambient_product  = GLState::light_ambient * material_ambient;
  color4 diffuse_product  = GLState::light_diffuse * material_diffuse;
  color4 specular_product = GLState::light_specular * material_specular;
  
  glUniform4fv( glGetUniformLocation(GLState::program, "AmbientProduct"), 1, ambient_product );
  glUniform4fv( glGetUniformLocation(GLState::program, "DiffuseProduct"), 1, diffuse_product );
  glUniform4fv( glGetUniformLocation(GLState::program, "SpecularProduct"), 1, specular_product );
  glUniform4fv( glGetUniformLocation(GLState::program, "LightPosition"), 1, lightPosition );
  glUniform1f(  glGetUniformLocation(GLState::program, "Shininess"), material_shininess );
  
  glBindVertexArray(vao);
  glBindBuffer( GL_ARRAY_BUFFER, buffer );
  glVertexAttribPointer( GLState::vPosition, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0) );
  glVertexAttribPointer( GLState::vNormal, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(object->mesh.vertices.size()*sizeof(vec4)) );
  
  mat4 objectModelView = GLState::sceneModelView*object->getModelView();
  
  
  glUniformMatrix4fv( GLState::ModelViewLight, 1, GL_TRUE, GLState::sceneModelView);
  glUniformMatrix3fv( GLState::NormalMatrix, 1, GL_TRUE, Normal(objectModelView));
  glUniformMatrix4fv( GLState::ModelView, 1, GL_TRUE, objectModelView);
  
  glDrawArrays( GL_TRIANGLES, 0, object->mesh.vertices.size() );
  
}


int main(void){
  
  GLFWwindow* window;
  
  glfwSetErrorCallback(error_callback);
  
  if (!glfwInit())
    exit(EXIT_FAILURE);
  
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  
  glfwWindowHint(GLFW_SAMPLES, 4);
  
  window = glfwCreateWindow(768, 768, "Raytracer", NULL, NULL);
  if (!window){
    glfwTerminate();
    exit(EXIT_FAILURE);
  }
  
  glfwSetKeyCallback(window, keyCallback);
  glfwSetMouseButtonCallback(window, mouseClick);
  glfwSetCursorPosCallback(window, mouseMove);

  
  glfwMakeContextCurrent(window);
  gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);
  glfwSwapInterval(1);
  
  initScene();
  initGL();
  
  while (!glfwWindowShouldClose(window)){
    
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);

    GLState::window_height = height;
    GLState::window_width  = width;

    glViewport(0, 0, width, height);
    
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    mat4 track_ball =  mat4(GLState::curmat[0][0], GLState::curmat[1][0],
                            GLState::curmat[2][0], GLState::curmat[3][0],
                            GLState::curmat[0][1], GLState::curmat[1][1],
                            GLState::curmat[2][1], GLState::curmat[3][1],
                            GLState::curmat[0][2], GLState::curmat[1][2],
                            GLState::curmat[2][2], GLState::curmat[3][2],
                            GLState::curmat[0][3], GLState::curmat[1][3],
                            GLState::curmat[2][3], GLState::curmat[3][3]);
 
   GLState::sceneModelView  =  Translate(-cameraPosition) *   //Move Camera Back
                               Translate(GLState::ortho_x, GLState::ortho_y, 0.0) *
                               track_ball *                   //Rotate Camera
                               Scale(GLState::scalefactor,
                                     GLState::scalefactor,
                                     GLState::scalefactor);   //User Scale
    
    GLfloat aspect = GLfloat(width)/height;
    GLState::projection = Perspective( 45.0, aspect, 4.5, 100.0 );
    
    glUniformMatrix4fv( GLState::Projection, 1, GL_TRUE, GLState::projection);
    
    for(unsigned int i=0; i < sceneObjects.size(); i++){
      drawObject(sceneObjects[i], GLState::objectVao[i], GLState::objectBuffer[i]);
    }
    
    glfwSwapBuffers(window);
    glfwPollEvents();
    
  }
  
  glfwDestroyWindow(window);
  
  glfwTerminate();
  exit(EXIT_SUCCESS);
}
