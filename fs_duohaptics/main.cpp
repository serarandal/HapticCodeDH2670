//===========================================================================
/*
 *
 *  KTH Collaborative Dual-User Haptic Environment Application
 *  Developed by Jonas Forsslund, Forsslund Systems AB
 *  December 2017: jonas@forsslundsystems.se
 *
 *  Partly based on the BSD-Licensed open source code of example
 *  "01-bullet-cube" released with the Chai3d-BULLET module
 *  of Chai3D version 3.2.0.
 *
 *  Collision event handling, colgroups and generic6dof-code based on
 *  Dickinson, Charles (2013) book "Learning Game Physics with
 *  Bullet Physics and OpenGL"
 *
 *
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CBullet.h"
#include <vector>
//---------------------------------------------------------------------------

// For bullets collision event handling
#include <set>
#include <iterator>
#include <algorithm>
#include <map>

#include <iomanip>
#include <random>




//---------------------------------------------------------------------------
// GENERAL SETTINGS
//---------------------------------------------------------------------------
// Select what method we should use for haptic rendering. Either we can use
// Bullet for everything (BULLET_FOR_AVATARS) or we can let Chai do the
// haptic rendering using their tool mechanism (TOOLS), but move other objects
// with Bullet.
#define TOOLS
//#define BULLET_FOR_AVATARS

// If using TOOLS, you may want to make all objects freeze as soon as they
// hit something, except when being picked up, and a short while
// after they have been relezed, so they can fall of gravity.
//#define USE_DELAYED_FREEZE

// If using BULLET_FOR_AVATARS you may want to exchange the generic6dof
// constraint for a spring one, which however gives inertia-ish feeling
//#define SPRING_6DOF

// For debug, show the actual device positions
//#define SHOW_DEVICE_SPHERES

// Ignore box-box interaction altogether
//#define USE_COLGROUPS

// Sometimes Bullet is happier working on a larger scale, i.e. it is
// said to be designed to work on objects in meters, while we like
// to work in sizes of centimeters (units of 0.01) to stay in 1:1 mapping
// of haptic devices. World can be scaled though, if stiffness etc for
// haptic rendering then is scaled down, motion scaled up, and gravity
// scaled as well (otherwise it will look like objects fall slowly).
// (this is feature is only partly implemented)
//#define USE_SCALE

// One way to limit boxes from being pushed around except for gravity
// is to limit all motions but z except for when they are picked up.
//#define LIMIT_FREE_MOTION_TO_Z

// This is the original strategy fo the Chai3d example, move boxes by applying
// forces equal but oppositie to that rendered to the haptic device. It
// allows for pushing around boxes, with nice friction.
// To get good feeling the force can be scaled in different directions.
#define HAPTIC_PUSH
cVector3d hapticPushFactors = cVector3d(0.1,0.1,0.4);

// Kinematic (in bullet) means boxes are moved by end programmer not by bullet
// in our case it means you can move about boxes but they do not fall...
//#define KINEMATIC

// ... unless we implement our own gravity
//#define MY_GRAVITY

// Use the OSC (Open Sound Control) feature, sending data over UDP of
// current positions, forces and collision events
#define OSC




// for open sound control
#ifdef OSC
#include "osc/osc/OscOutboundPacketStream.h"
#include "osc/ip/UdpSocket.h"
#define ADDRESS "127.0.0.1"
#define PORT 7000
#define OUTPUT_BUFFER_SIZE 1024
UdpTransmitSocket *transmitSocket;
#endif


// stereo Mode
//cStereoMode stereoMode = C_STEREO_DISABLED; //
cStereoMode stereoMode = C_STEREO_ACTIVE;     // 3D vision on GeForce will
                                              // render as one "eye" only
                                              // in window mode, but full 3D
                                              // in fullscreen (press "F" key)
                                              // and press "M" for mirror



//------------------------------------------------------------------------------
// Simulation Settings
//------------------------------------------------------------------------------
bool disable_haptics[] = {false,false}; // Disable forces for respective device

double boxMass       = 0.05; // kg for simulation. Increasing this to e.g. 5kg
                             // will make boxes almost static except for
                             // gravity
double temporaryLowMass = 0.0005; // 0.0005 nice
double boxHapticMass = 0.05; // kg for haptic gravity (how heavy boxes feel)
double boxSize       = 0.02; // meter side
double avatarRadius  = 0.0025; // m radius of avatars (and debug spheres)

double camera_x=0.26; // Make it 0.36 to have object be 1:1 on screen surface
                      // 0.26 is nice for normal screen.
double camera_z=0.04; // 0.04 is nice for normal screen

double stiffness = 700; // N/m
double manipulandumGravityCompensation = 0.2; // N

int maxBoxes = 26; // English alphabet (textures) limit

#ifdef USE_SCALE
double scale=100.0; //Bullet&Chai units 100 times reality, i.e. 1cm reality is 1m
#else
double scale=1.0;
#endif





//---------------------------------------------------------------------------
// CHAI3D VARIABLES
//---------------------------------------------------------------------------
bool fullscreen = false;
bool mirroredDisplay = false;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice[2];
int numDevices=0;

// a virtual tool representing the haptic device in the scene
#ifdef TOOLS
cGenericTool* tool[] = {0,0};
#endif

#ifdef MY_GRAVITY
std::map<cBulletBox*,double> gravityVelocity;
std::map<cBulletBox*,bool>   gravityGo;
//std::map<btRigidBody*,std::vector<btRigidBody*>>  ;
#endif


cShapeSphere* deviceSphere[] = {0,0};

bool requestAddBox=false;
int requestClose=4;

// a label to display the rates [Hz] at which the simulation is running
cLabel* labelRates;

bool currentButton[] = {false,false};
bool previousButton[] = {false,false};
bool requestedPickup[] = {false,false};
bool requestedRelease[] = {false,false};


bool holdOther[] = {false,false};

std::map<void*,std::string> namemap;

// Our own wait for small forces, since we cant get the state from tool
double smallForceThresh = 0.2;
bool engageForces[] = {false,false};
int smallForceCounter[] = {0,0};
cVector3d prevForce[] = {cVector3d(0,0,0), cVector3d(0,0,0)};

//---------------------------------------------------------------------------
// BULLET MODULE VARIABLES
//---------------------------------------------------------------------------

// bullet world
cBulletWorld* bulletWorld;


class cBulletBoxDeluxe : public cBulletBox {
  public:
    cBulletBoxDeluxe(cBulletWorld* a_world,
        const double& a_sizeX,
        const double& a_sizeY,
        const double& a_sizeZ):cBulletBox(a_world,a_sizeX,a_sizeY,a_sizeZ){}
    std::string name;
};

std::vector<cBulletBoxDeluxe*> bulletBoxes;
int numBoxes = 0;

// bullet static walls and ground
cBulletStaticPlane* bulletInvisibleWall1;
cBulletStaticPlane* bulletInvisibleWall2;
cBulletStaticPlane* bulletInvisibleWall3;
cBulletStaticPlane* bulletInvisibleWall4;
cBulletStaticPlane* bulletInvisibleWall5;
cBulletStaticPlane* bulletGround;

// For picking up objects
btGeneric6DofConstraint* dof6[] = {0,0};
cShapeSphere* avatarVisuals[2] = {0,0};

#ifdef BULLET_FOR_AVATARS
#ifdef SPRING_6DOF
btGeneric6DofSpring2Constraint* acon[2] = {0,0};
#else
btGeneric6DofConstraint* acon[2] = {0,0};
#endif
cBulletSphere* avatar[2] = {0,0};
cVector3d applied_f[2] = { cVector3d(0,0,0), cVector3d(0,0,0)};
#endif

//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

#ifdef USE_COLGROUPS
enum CollisionGroups {
    COLGROUP_NONE   = 0,
    COLGROUP_STATIC = 1 << 0,
    COLGROUP_BOX    = 1 << 1,
    COLGROUP_SPHERE = 1 << 2
};
#endif

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// for random number generation
std::default_random_engine generator;
std::uniform_real_distribution<double> distribution(0.0,1.0);

// Bullet collision event handling
typedef std::pair<const btRigidBody*, const btRigidBody*> CollisionPair;
typedef std::set<CollisionPair> CollisionPairs;
CollisionPairs m_pairsLastUpdate;

// when using scaling
double workspaceScaleFactor=1.0;





//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

// Convert vectors between bullet and chai
btVector3 c2b(const cVector3d& c) { return btVector3(c.x(),c.y(),c.z()); }
cVector3d b2c(const btVector3& b) { return cVector3d(b.x(),b.y(), b.z()); }

// Pickup box function and related
void pickup(btRigidBody* pickedBody, int deviceIndex);
void freeze(btRigidBody* r);
void unfreeze(btRigidBody* r, double delay=0);
bool isHeld(btRigidBody *pBody);

// Collision (in bullet) event handling
void CheckForCollisionEvents();
void CollisionEvent(btRigidBody* pBody0, btRigidBody* pBody1);
void SeparationEvent(btRigidBody* pBody0, btRigidBody* pBody1);

// Find our boxes giving their rigid body
cBulletBox* FindChaiObject(btRigidBody *body);

// Add a new box to the scene
void addBox();

void printNice(cVector3d v){
    std::cout << std::setprecision(3) << std::fixed;
    std::cout << "      " << std::setw(6) << v.x() << ", "
                          << std::setw(6) << v.y() << ", "
                          << std::setw(6) << v.z();
}

//------------------------------------------------------------------------------
// Throw collision events, from Bullet book
//------------------------------------------------------------------------------
void CheckForCollisionEvents() {
    // keep a list of the collision pairs we found during the current update
    CollisionPairs pairsThisUpdate;

    btDispatcher* m_pDispatcher = bulletWorld->m_bulletWorld->getDispatcher();
    for(int i=0;i<m_pDispatcher->getNumManifolds();++i){
        // get the manifold
        btPersistentManifold* pManifold = m_pDispatcher->getManifoldByIndexInternal(i);

        // ignore manifolds that have no contact points
        if(pManifold->getNumContacts()>0){
            // get the two rigid bodies involved in the collision
            const btRigidBody* pBody0 = static_cast<const btRigidBody*>(pManifold->getBody0());
            const btRigidBody* pBody1 = static_cast<const btRigidBody*>(pManifold->getBody1());

            // always create the pair in a predicatable order (use the pointer value...)
            bool const swapped = pBody0 > pBody1;
            const btRigidBody* pSortedBodyA = swapped? pBody1: pBody0;
            const btRigidBody* pSortedBodyB = swapped? pBody0: pBody1;

            // create the pair
            CollisionPair thisPair = std::make_pair(pSortedBodyA, pSortedBodyB);

            // insert
            pairsThisUpdate.insert(thisPair);

            // if this pair doesn't exist in the list from the previous update, it is a new
            // pair and we must send a collision event!
            if(m_pairsLastUpdate.find(thisPair) == m_pairsLastUpdate.end())
                CollisionEvent((btRigidBody*)(pBody0), (btRigidBody*)(pBody1));
        }
    }

    // create another list for pairs that were removed this update
    CollisionPairs removedPairs;

    // this handy function gets the difference between two sets. It takes
    // the difference between collision pairs from the last update, and this
    // update, and pushes them into the removed pairs list.
    std::set_difference(m_pairsLastUpdate.begin(), m_pairsLastUpdate.end(),
                        pairsThisUpdate.begin(), pairsThisUpdate.end(),
                        std::inserter(removedPairs, removedPairs.begin()));

    // iterate through all of the removed pairs sending separation events for them
    for(CollisionPairs::const_iterator iter = removedPairs.begin(); iter!=removedPairs.end();++iter){
        SeparationEvent((btRigidBody*)(iter->first), (btRigidBody*)(iter->second));
    }

    // in the next iteration we'll want to compare against the pairs we found just now
    m_pairsLastUpdate = pairsThisUpdate;
}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
cBulletBox* FindChaiObject(btRigidBody *body){
    for(auto p:bulletBoxes)
        if(p->m_bulletRigidBody==body) return p;
    return 0;
}
//------------------------------------------------------------------------------




//------------------------------------------------------------------------------
bool isHeld(btRigidBody *pBody){
    for(int t=0;t<numDevices;++t){
        if(!dof6[t]) continue;
        if(&(dof6[t]->getRigidBodyB()) == pBody)
            return true;
    }
    return false;
}
bool isHeldChai(cGenericObject *pBody){
    if(cBulletGenericObject* bobj = dynamic_cast<cBulletGenericObject*>(pBody))
        return(isHeld(bobj->m_bulletRigidBody));
    return false;
}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
void CollisionEvent(btRigidBody *pBody0, btRigidBody *pBody1){

#ifdef KINEMATIC
    //gravityGo[FindChaiObject(pBody0)] = false;
    //gravityGo[FindChaiObject(pBody1)] = false;
#endif

#ifdef BULLET_FOR_AVATARS
    for(int t=0;t<numDevices;++t){
        if(!currentButton[t]) continue;
        if(pBody0==avatar[t]->m_bulletRigidBody)
            pickup(pBody1,t);
        if(pBody1==avatar[t]->m_bulletRigidBody)
            pickup(pBody0,t);
    }
#endif

    // find the two colliding objects
    cGenericObject* pObj0 = FindChaiObject(pBody0);
    cGenericObject* pObj1 = FindChaiObject(pBody1);

    /*
    if(auto b = dynamic_cast<cBulletBoxDeluxe*>(pObj0)){
        std::cout << b->name << " collided with ";
    } else std::cout << "something collided with ";
    if(auto b = dynamic_cast<cBulletBoxDeluxe*>(pObj1)){
        std::cout << b->name << "\n";
    } else std::cout << "something\n";*/
    std::cout << namemap[pBody0] << " collides with " << namemap[pBody1] <<"\n";

#ifdef OSC
    // Send OSC data
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);
    p.Clear();
    p << osc::BeginMessage("/events") << (namemap[pBody0]).c_str() << " collides with "
      << (namemap[pBody1]).c_str() << osc::EndMessage;

    transmitSocket->Send( p.Data(), p.Size() );
#endif



    // exit if we didn't find anything
    if(!pObj0 || !pObj1) return;

    // Now we have 2 bulletboxes objects

#ifdef USE_DELAYED_FREEZE
    if(isHeldChai(pObj0) && !isHeldChai(pObj1))
        freeze(pBody1);
    if(isHeldChai(pObj1) && !isHeldChai(pObj0))
        freeze(pBody0);
#endif














}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
void SeparationEvent(btRigidBody *pBody0, btRigidBody *pBody1){

#ifdef KINEMATIC
    //gravityGo[FindChaiObject(pBody0)] = true;
    //gravityGo[FindChaiObject(pBody1)] = true;
#endif


    std::cout << namemap[pBody0] << " separates from " << namemap[pBody1] <<"\n";

#ifdef OSC
    // Send OSC data
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);
    p.Clear();
    p << osc::BeginMessage("/events") << (namemap[pBody0]).c_str() << " separates from "
      << (namemap[pBody1]).c_str() << osc::EndMessage;

    transmitSocket->Send( p.Data(), p.Size() );
#endif

    // get the two separating objects
    cGenericObject* pObj0 = FindChaiObject(pBody0);
    cGenericObject* pObj1 = FindChaiObject(pBody1);

    // exit if we didn't find anything
    if(!pObj0 || !pObj1) return;

#ifdef USE_DELAYED_FREEZE
    if(isHeldChai(pObj0) && !isHeldChai(pObj1))
        unfreeze(pBody1,0.01);
    if(isHeldChai(pObj1) && !isHeldChai(pObj0))
        unfreeze(pBody0,0.01);
#endif
}
//------------------------------------------------------------------------------






//------------------------------------------------------------------------------
#ifdef BULLET_FOR_AVATARS
void pair_avatar(btRigidBody* rigid, cGenericHapticDevice *device, int device_index=0){
    rigid->setActivationState(DISABLE_DEACTIVATION);
    cVector3d pos;
    pos.zero();
    //device->getPosition(pos);
    btTransform t;
    t.setIdentity();
    t.setOrigin(c2b(pos));

#ifdef SPRING_6DOF
    // From bullet example collection
    acon[device_index] = new btGeneric6DofSpring2Constraint(*rigid, t);
    acon[device_index]->setLimit(0, 0, 0);
    acon[device_index]->setLimit(1, 0, 0);
    acon[device_index]->setLimit(2, 0, 0);
    acon[device_index]->setLimit(3, 0, 0);
    acon[device_index]->setLimit(4, 0, 0);
    acon[device_index]->setLimit(5, 0, 0);
    acon[device_index]->enableSpring(0, true);
    acon[device_index]->setStiffness(0, stiffness);
    acon[device_index]->setDamping(0, 0);
    acon[device_index]->setEquilibriumPoint(0, 0);
    acon[device_index]->setDbgDrawSize(btScalar(2.f));
#else
    acon[device_index] = new btGeneric6DofConstraint(*rigid, t, true);
    acon[device_index]->setAngularLowerLimit(btVector3(0,0,0));
    acon[device_index]->setAngularUpperLimit(btVector3(0,0,0));
    float cfm = 0.2f;
    float erp = 0.5f; // error reduction
    for(int i=0;i<6;++i){
        acon[device_index]->setParam(BT_CONSTRAINT_STOP_CFM,cfm,i);
        acon[device_index]->setParam(BT_CONSTRAINT_STOP_ERP,erp,i);
    }
#endif


    // add the constraint to the world
    bulletWorld->m_bulletWorld->addConstraint(acon[device_index],true);
}
#endif
//------------------------------------------------------------------------------




//------------------------------------------------------------------------------
// Timeout event handling (for delay freezing)
//------------------------------------------------------------------------------
cPrecisionClock wall_clock;
struct timeout_event {
    double when;
    btRigidBody* body;
};
std::vector<timeout_event> timeout_events;
void tick_wall_clock_events(){
    double now = wall_clock.getCurrentTimeSeconds();
    for(std::vector<timeout_event>::iterator itr=timeout_events.begin();
        itr!=timeout_events.end();){
        timeout_event& t = *itr;
        if(now>(t.when)){
            unfreeze(t.body);
            itr = timeout_events.erase(itr);
        } else
            itr++;
    }
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void freeze(btRigidBody* r){
    //std::cout << "freeze\n";
    btScalar newMass = 0; // make static
    bulletWorld->m_bulletWorld->removeRigidBody(r);
    btVector3 inertia;
    r->getCollisionShape()->calculateLocalInertia(newMass,inertia);
    r->setMassProps(newMass, inertia);
#ifdef KINEMATIC
    r->setCollisionFlags( r->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    r->setActivationState(DISABLE_DEACTIVATION);
#endif

    r->clearForces();
    btVector3 zeroVector(0,0,0);
    r->setLinearVelocity(zeroVector);

    bulletWorld->m_bulletWorld->addRigidBody(r);
}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
void unfreeze(btRigidBody* r, double delay){
    //std::cout << "Unfreeze " << delay << " \n";
    if(delay>0){
        timeout_event t;
        t.body=r;
        t.when=wall_clock.getCurrentTimeSeconds()+delay;
        timeout_events.push_back(t);
        return;
    }

    btScalar newMass = boxMass; // make dynamic
    bulletWorld->m_bulletWorld->removeRigidBody(r);
    btVector3 inertia;

    r->getCollisionShape()->calculateLocalInertia(newMass,inertia);
    r->setMassProps(newMass, inertia);
    r->clearForces();
    btVector3 zeroVector(0,0,0);
    r->setLinearVelocity(zeroVector);

    bulletWorld->m_bulletWorld->addRigidBody(r);
}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
void pickup(btRigidBody* pickedBody, int deviceIndex){
    if(dof6[deviceIndex]) return;
    if(!FindChaiObject(pickedBody)) return; // Only pickup boxes

    std::cout << "Pickup with device " << deviceIndex << "\n";

    requestedPickup[deviceIndex] = false;


#ifdef LIMIT_FREE_MOTION_TO_Z
    m_pPickedBody->setLinearFactor(btVector3(1,1,1));
#endif

#ifdef KINEMATIC
    m_pPickedBody->setCollisionFlags(0);
    m_pPickedBody->setActivationState(ACTIVE_TAG);
    unfreeze(m_pPickedBody);
#endif


    // Make object light temporarily, this helps rendering friction
    // when pushing around another box with the picked up box.
    bulletWorld->m_bulletWorld->removeRigidBody(pickedBody);
    btScalar newMass = temporaryLowMass;
    btVector3 inertia;
    pickedBody->getCollisionShape()->calculateLocalInertia(newMass,inertia);
    pickedBody->setMassProps(newMass, inertia);
    bulletWorld->m_bulletWorld->addRigidBody(pickedBody);


    /* Change colgroup */
#ifdef USE_COLGROUPS
    bulletWorld->m_bulletWorld->removeRigidBody(m_pPickedBody);
    bulletWorld->m_bulletWorld->addRigidBody(m_pPickedBody,COLGROUP_BOX, COLGROUP_STATIC);
#endif

    // Remove from chai collision detection
    /*
    for(int i=0;i<bulletBoxes.size();++i){
        if(bulletBoxes[i]->m_bulletRigidBody == m_pPickedBody)
           bulletBoxes[i]->deleteCollisionDetector();
    }*/


    // ***************** pickup code from bullet book **********
    pickedBody->setActivationState(DISABLE_DEACTIVATION);
    btVector3 p = pickedBody->getCenterOfMassTransform().getOrigin();
#ifdef TOOLS
    cVector3d t = tool[deviceIndex]->m_image->getGlobalPos();
#endif
#ifdef BULLET_FOR_AVATARS
    cVector3d t = avatar[deviceIndex]->getGlobalPos();
#endif

    cVector3d diff = t - b2c(p);
    cVector3d dir=diff;
    dir.normalize();
    cVector3d cpivot = diff;// + dir*0.01*scale; // So we are not in contact


    // Set visual avatar to fixed position on surface of picked object
    cGenericObject *obj = FindChaiObject(pickedBody);
#ifdef BULLET_FOR_AVATARS
    avatar[deviceIndex]->removeChild(avatarVisuals[deviceIndex]);
#else
    tool[deviceIndex]->m_image->removeChild(avatarVisuals[deviceIndex]);
#endif
    obj->addChild(avatarVisuals[deviceIndex]);
    avatarVisuals[deviceIndex]->setLocalPos(diff);



/*
    std::cout << "t         : " << t << "\n";
    std::cout << "p         : " << b2c(p) << "\n";
    std::cout << "LocalPivot: " << cpivot << "\n";
*/

    // create a transform for the pivot point
    btTransform pivot;
    pivot.setIdentity();
    pivot.setOrigin(c2b(cpivot));

    // create our constraint object
    dof6[deviceIndex] = new btGeneric6DofConstraint(*pickedBody, pivot, true);
    dof6[deviceIndex]->enableFeedback(true);
    bool bLimitAngularMotion = true;
    if(bLimitAngularMotion) {
        dof6[deviceIndex]->setAngularLowerLimit(btVector3(0,0,0));
        dof6[deviceIndex]->setAngularUpperLimit(btVector3(0,0,0));
    }
#ifdef USE_DELAYED_FREEZE
    unfreeze(m_pPickedBody);
#endif

    // define the 'strength' of our constraint (each axis)
    float cfm = 0.9f;
    float erp = 0.1f; // error reduction
    for(int i=0;i<6;++i){
        dof6[deviceIndex]->setParam(BT_CONSTRAINT_STOP_CFM,cfm,i);
        dof6[deviceIndex]->setParam(BT_CONSTRAINT_STOP_ERP,erp,i);
    }

    // add the constraint to the world
    bulletWorld->m_bulletWorld->addConstraint(dof6[deviceIndex],true);


#ifdef OSC
    // Send OSC data
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream op(buffer, OUTPUT_BUFFER_SIZE);
    op.Clear();
    op << osc::BeginMessage("/events") << std::to_string(deviceIndex).c_str() << " picks up "
      << (namemap[pickedBody]).c_str() << osc::EndMessage;
    transmitSocket->Send( op.Data(), op.Size() );
#endif

}
//------------------------------------------------------------------------------




//------------------------------------------------------------------------------
void release(int deviceIndex){
    if(!dof6[deviceIndex]) return;

    std::cout << "Release with device " << deviceIndex << "\n";   

    //m_pPickedBody->setActivationState(DISABLE_DEACTIVATION); //TODO: ENABLE
#ifdef USE_COLGROUPS
    btRigidBody* m_pPickedBody = &(dof6[deviceIndex]->getRigidBodyB());
    bulletWorld->m_bulletWorld->removeRigidBody(m_pPickedBody);
    bulletWorld->m_bulletWorld->addRigidBody(m_pPickedBody,COLGROUP_BOX, COLGROUP_BOX | COLGROUP_STATIC);
#endif

#ifdef LIMIT_FREE_MOTION_TO_Z
    // Unless we hold same object, release
    if(!(dof6[0] && dof6[1] && &(dof6[0]->getRigidBodyB()) == &(dof6[1]->getRigidBodyB()))){
        dof6[deviceIndex]->getRigidBodyB().setLinearFactor(btVector3(0,0,1));
        dof6[deviceIndex]->getRigidBodyB().setLinearVelocity(btVector3(0,0,0));
    }
#endif

    // make sleepable
    btRigidBody* pickedBody = &dof6[deviceIndex]->getRigidBodyB();
    pickedBody->setActivationState(ACTIVE_TAG);

    // Restore mass
    bulletWorld->m_bulletWorld->removeRigidBody(pickedBody);
    btScalar newMass = boxMass;
    btVector3 inertia;
    pickedBody->getCollisionShape()->calculateLocalInertia(newMass,inertia);
    pickedBody->setMassProps(newMass, inertia);
    bulletWorld->m_bulletWorld->addRigidBody(pickedBody);

#ifdef KINEMATIC
    if(!(dof6[0] && dof6[1] && &dof6[0]->getRigidBodyB() == &dof6[1]->getRigidBodyB()))
        freeze(&dof6[deviceIndex]->getRigidBodyB());
#endif

    // Enable haptics
    for(int b=0;b<bulletBoxes.size();++b){
        if(bulletBoxes[b]->m_bulletRigidBody == pickedBody){
            bulletBoxes[b]->setHapticEnabled(true);

#ifdef TOOLS
            // Move proxy
            if(cToolCursor* tc=dynamic_cast<cToolCursor*>(tool[deviceIndex])){
                tc->m_hapticPoint->m_algorithmFingerProxy->setProxyGlobalPosition(
                            avatarVisuals[deviceIndex]->getGlobalPos());
            }
#endif

            // Give visual avatar back to avatar mover
            bulletBoxes[b]->removeChild(avatarVisuals[deviceIndex]);
#ifdef BULLET_FOR_AVATARS
            avatar[deviceIndex]->addChild(avatarVisuals[deviceIndex]);
#else
            tool[deviceIndex]->m_image->addChild(avatarVisuals[deviceIndex]);
#endif
            avatarVisuals[deviceIndex]->setLocalPos(0,0,0);

            break;
        }
    }

    requestedRelease[deviceIndex]=false;
    bulletWorld->m_bulletWorld->removeConstraint(dof6[deviceIndex]);
    delete dof6[deviceIndex];
    dof6[deviceIndex]=0;

#ifdef OSC
    // Send OSC data
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);
    p.Clear();
    p << osc::BeginMessage("/events") << std::to_string(deviceIndex).c_str() << " releases "
      << (namemap[pickedBody]).c_str() << osc::EndMessage;
    transmitSocket->Send( p.Data(), p.Size() );
#endif
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// For our information. However, we will not get all forces this way, only
// sum of externally applied forces (like gravity). But not forces resulting
// from e.g. stacking boxes on top of the current object. This is due to the
// fact that Bullet uses constraint resolvers rather than explicit dynamic
// integration.
//------------------------------------------------------------------------------
void myTickCallback(btDynamicsWorld *world, btScalar timeStep) {
    //std::cout << "The world just ticked by " << double(timeStep) << " seconds\n";

#ifdef MY_GRAVITY
    //CheckForCollisionEvents();
    for(auto b:bulletBoxes){
        bool inContact=false;
        /*
        // in contact with anything?
        for(auto c:bulletBoxes){
            if(b==c) continue;
            if(b->m_bulletRigidBody->checkCollideWith(c->m_bulletRigidBody)){
                inContact=true;
                break;
            }
        }*/


        for(auto cp : m_pairsLastUpdate){
            if(cp.first == b->m_bulletRigidBody || cp.second == b->m_bulletRigidBody){
                inContact=true;
                break;
            }
        }

        if(inContact) gravityGo[b]=false;
        else gravityGo[b]=true;
    }

    for(auto b:bulletBoxes){

        if(gravityGo[b]){
            gravityVelocity[b]=-0.2;
        } else
            gravityVelocity[b]=0.0;

//        unfreeze(b->m_bulletRigidBody);
//        bulletWorld->computeGlobalPositions(true);
        //b->m_bulletRigidBody->translate(btVector3(0,0,-0.01));
        btTransform t;
        b->m_bulletMotionState->getWorldTransform(t);
        if(t.getOrigin().z()>0.0125)
            t.setOrigin(t.getOrigin()+btVector3(0,0,gravityVelocity[b]*timeStep));
        b->m_bulletMotionState->setWorldTransform(t);
//        bulletWorld->computeGlobalPositions(true);
//        freeze(b->m_bulletRigidBody);
    }
#endif

#ifdef BULLET_FOR_AVATARS
    for(int i=0;i<2;++i)
        applied_f[i] = b2c(avatar[i]->m_bulletRigidBody->getTotalForce());
#endif
}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Coloring of boxes
//------------------------------------------------------------------------------
cColorf hex2rgb(int hex){
    float b = float((hex & 0x0000FF) >> 0)/255.0f;
    float g = float((hex & 0x00FF00) >> 8)/255.0f;
    float r = float((hex & 0xFF0000) >> 16)/255.0f;
    return cColorf(r,g,b);
}
int m_nextcolor = 0;
cColorf colors[] = {   hex2rgb(0x7b68ee),
                       hex2rgb(0x98fb98),
                       hex2rgb(0xffffe0),
                       hex2rgb(0xcd5c5c),
                       hex2rgb(0xffa07a),
                       hex2rgb(0xee82ee),
                       hex2rgb(0xffd),
                       hex2rgb(0x9acd32),
                       hex2rgb(0xfa8072),
                       hex2rgb(0xffefd5),
                       hex2rgb(0xf8f8ff),
                       hex2rgb(0xfffaf0),
                       hex2rgb(0xeed5b7),
                       hex2rgb(0xffc0cb),
                       hex2rgb(0xffa07a)
                     };
//------------------------------------------------------------------------------




//------------------------------------------------------------------------------
// Define our interactive boxes
//------------------------------------------------------------------------------
void addBox(){
    if(numBoxes+1>maxBoxes) return;
    double maxStiffness	= stiffness;
#ifdef USE_SCALE
    maxStiffness /= workspaceScaleFactor;
#endif

    cMaterial mat;
    mat.setGreenDarkSea();
    mat.setStiffness(maxStiffness);
    mat.setDynamicFriction(0.1);
    mat.setStaticFriction(0.1);


    double size = boxSize;
#ifdef USE_SCALE
    size*=scale;
#endif
    cBulletBoxDeluxe* b = new cBulletBoxDeluxe(bulletWorld, size, size, size);
    m_nextcolor = m_nextcolor>=15? 0 : m_nextcolor+1;
    mat.setColor(colors[m_nextcolor]);
    b->setMaterial(mat);

    double x = distribution(generator)*0.075-0.04+0.02/2.0;
    double y = distribution(generator)*0.15-0.075;
    double z = 0.1-boxSize;
#ifdef KINEMATIC
    z = distribution(generator)*0.1-0.05;
#endif

#ifdef USE_SCALE
    x*=scale;
    y*=scale;
    z*=scale;
#endif

    b->setLocalPos(x, y, z);

    // Texture
    b->m_texture = cTexture2d::create();
    b->setUseTexture(true);
    std::string filename = "slab_" + std::string(1,('a'+numBoxes%26)) + ".png";
    b->m_texture->loadFromFile(filename);

#ifdef KINEMATIC
    b->setMass(0);
#else
    b->setMass(boxMass);
#endif

    b->estimateInertia();
#ifdef USE_COLGROUPS
    b->m_bulletGroup = COLGROUP_BOX;
    b->m_bulletMask = COLGROUP_BOX | COLGROUP_STATIC;
#endif
    b->buildDynamicModel();
    b->m_bulletRigidBody->setAngularFactor(btVector3(0,0,0));
#ifdef LIMIT_FREE_MOTION_TO_Z
    b->m_bulletRigidBody->setLinearFactor(btVector3(0,0,1));
#endif

#ifdef KINEMATIC
    b->m_bulletRigidBody->setCollisionFlags( b->m_bulletRigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    b->m_bulletRigidBody->setActivationState(DISABLE_DEACTIVATION);
#endif
    b->m_bulletRigidBody->setFriction(0.9);
    b->m_bulletRigidBody->setDamping(0.2,0);


    b->m_bulletRigidBody->setRestitution(0.5);
    b->m_bulletRigidBody->setActivationState(ACTIVE_TAG);
    //b->m_bulletRigidBody->setSleepingThresholds(1,1);

#ifdef TOOLS
    b->createAABBCollisionDetector(avatarRadius);
    b->setSurfaceFriction(0.2);
#endif

    b->setDamping(0.9,0);

    // name
    b->name = std::string(1,('A'+numBoxes%26));
    namemap[b->m_bulletRigidBody] = b->name;


    bulletWorld->addChild(b);
    bulletBoxes.push_back(b);
#ifdef MY_GRAVITY
    gravityVelocity[b] = 0;
    gravityGo[b] = true;
#endif
    numBoxes++;
}
//------------------------------------------------------------------------------











//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    cout << endl;
    cout << "------------------------------------" << endl;
    cout << "KTH Duohaptics application, (c) 2017" << endl;
    cout << "------------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[space] - Add a new box" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable mirror mode" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;


    // OPEN SOUND CONTROL
#ifdef OSC
    transmitSocket = new UdpTransmitSocket( IpEndpointName( ADDRESS, PORT ) );
#endif

    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = int(0.8 * mode->height);
    int h = int(0.5 * mode->height);
    int x = int(0.5 * (mode->width - w));
    int y = int(0.5 * (mode->height - h));

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

    // initialize GLEW library
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //-----------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //-----------------------------------------------------------------------

    // create a dynamic world.
    bulletWorld = new cBulletWorld();
    bulletWorld->m_bulletWorld->setInternalTickCallback(myTickCallback,0,true);

    // Specific configuration of the bullet dynamics integrator,
    // especially important since we have "small" objects
#ifdef USE_SCALE
    bulletWorld->setIntegrationSettings(0.0004,50);
#else
    bulletWorld->setIntegrationSettings(0.0002,25);
#endif

    // set the background color of the environment
    bulletWorld->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(bulletWorld);
    bulletWorld->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d (camera_x*scale,
                           0.0,
                           camera_z*scale), // camera position (eye) // 0.36=reality
                cVector3d (0.0, 0.0, 0.02), // lookat position (target)
                cVector3d (0.0, 0.0, 1.0)); // direction of the "up" vector


    // position and orient the camera
    camera->set(cVector3d (camera_x*scale,
                           0.0,
                           camera_z*scale), // camera position (eye) // 0.36=reality
                cVector3d (0.0, 0.0, 0.02), // lookat position (target)
                cVector3d (0.0, 0.0, 1.0)); // direction of the "up" vector


    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 1000.0);

    std::cout << "FOV: " << camera->getFieldViewAngleDeg() <<"\n";
    //camera->setFieldViewAngleDeg(55.84); // Default is 45

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.003);
    camera->setStereoFocalLength(1*scale);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cSpotLight(bulletWorld);

    // attach light to camera
    bulletWorld->addChild(light);

    // enable light source
    light->setEnabled(true);

    // position the light source
    light->setLocalPos(0.2, 0.0, 0.4*scale);

    // define the direction of the light beam
    light->setDir(-0.5, 0.0,-1.0);

    // set uniform concentration level of light 
    light->setSpotExponent(0.0);

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    light->m_shadowMap->setQualityVeryHigh();
    //light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    light->setCutOffAngleDeg(45);

    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    numDevices = cMax(handler->getNumDevices(),unsigned int(2));

    // get access to the first 2 available haptic device found
    for(int i=0;i<numDevices;++i){
        handler->getDevice(hapticDevice[i], i);
        hapticDevice[i]->open();
    }

    // retrieve information about the FIRST haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice[0]->getSpecifications();

#ifdef USE_SCALE
    avatarRadius *= scale;
#endif

#ifdef TOOLS
    for(int i=0;i<numDevices;++i){
        cGenericTool* t = new cToolCursor(bulletWorld);
        tool[i] = t;
        bulletWorld->addChild(t);

        // connect the haptic device to the virtual tool
        t->setHapticDevice(hapticDevice[i]);

        // map the physical workspace of the haptic device to a larger virtual workspace.
#ifdef USE_SCALE
        t->setWorkspaceRadius(0.13*scale);
        t->setRadius(avatarRadius+0.001*scale, avatarRadius); // Visual, Haptic
#else
        t->setWorkspaceRadius(hapticDeviceInfo.m_workspaceRadius); // make it 1:1
        // define a radius for the virtual tool contact points (sphere)
        t->setRadius(avatarRadius, avatarRadius); // Visual, Haptic
#endif


        // ignored anyway
        //t->setShowCollisionDetector(true);
        //t->setShowContactPoints(true);

        // enable if objects in the scene are going to rotate of translate
        // or possibly collide against the tool. If the environment
        // is entirely static, you can set this parameter to "false"
        t->enableDynamicObjects(true);

        // haptic forces are enabled only if small forces are first sent to the device;
        // this mode avoids the force spike that occurs when the application starts when
        // the tool is located inside an object for instance.
        t->setWaitForSmallForce(false); // We use our own, since we want to add
                                        // constant force (fake grav comp.) as well.

        // start the haptic tool
        t->start();
    }
#endif


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelRates);


    //-----------------------------------------------------------------------
    // SETUP BULLET WORLD AND OBJECTS
    //-----------------------------------------------------------------------

    //////////////////////////////////////////////////////////////////////////
    // BULLET WORLD
    //////////////////////////////////////////////////////////////////////////

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
#ifdef USE_SCALE
    workspaceScaleFactor = tool[0]->getWorkspaceScaleFactor();
    std::cout << "workspaceScaleFactor: " << workspaceScaleFactor;
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
    // set some gravity
    bulletWorld->setGravity(0.0, 0.0,-9.82*scale); // Sweden :)
#else

    // set some gravity
    bulletWorld->setGravity(0.0, 0.0,-9.82); // Sweden :)
#endif


    // Avatar material
    cMaterial matAvatar;
    matAvatar.setYellowGold();
    matAvatar.setStiffness(0.3 * stiffness);
    matAvatar.setDynamicFriction(0.6);
    matAvatar.setStaticFriction(0.6); // For chai

    // Make a visual only sphere for show when holding boxes
    for(int i=0;i<numDevices;++i){
        avatarVisuals[i] = new cShapeSphere(avatarRadius*1.1);
        avatarVisuals[i]->setHapticEnabled(false);
        avatarVisuals[i]->setMaterial(matAvatar);
        avatarVisuals[i]->m_material->setGreenLawn();
#ifdef TOOLS
        tool[i]->m_image->addChild(avatarVisuals[i]);
        tool[i]->getHapticPoint(0)->setShow(false,false);
#endif
    }


#ifdef BULLET_FOR_AVATARS
    // stiffness properties
    double maxStiffness	= stiffness;
    for(int i=0;i<numDevices;++i){
        avatar[i] = new cBulletSphere(bulletWorld,avatarRadius);
        bulletWorld->addChild(avatar[i]);
        avatar[i]->setMaterial(matAvatar);
        avatar[i]->setMass(0.001);
        avatar[i]->estimateInertia();
        avatar[i]->buildDynamicModel();

        avatar[i]->setLocalPos(0.05,0.0+i*0.05,0.04);

        avatar[i]->m_bulletRigidBody->setAngularFactor(0);
        avatar[i]->m_bulletRigidBody->setFriction(0.9);

        pair_avatar(avatar[i]->m_bulletRigidBody,hapticDevice[i].get(),i);
        avatar[i]->addChild(avatarVisuals[i]);
        avatar[i]->setShowEnabled(false,false);
    }
#endif

    // deviceSpheres (for debug)
    for(int i=0;i<numDevices;++i){
        cMaterial mat;
        mat.setYellowGold();
        //mat.setTransparencyLevel(0.9f);
        deviceSphere[i] = new cShapeSphere(avatarRadius*0.95);
        //deviceSphere[i]->setUseTransparency(true);
        deviceSphere[i]->setMaterial(mat);

#ifdef SHOW_DEVICE_SPHERES
        bulletWorld->addChild(deviceSphere[i]);
#endif
    }

    // CENTRAL SPHERES EXCATLY 10 CM APART (for testing screen size)
    /*
    for(int i=0;i<2;++i){
        cShapeSphere* s = new cShapeSphere(0.001);
        s->m_material->setBlue();
        bulletWorld->addChild(s);
        s->setLocalPos(0,-0.05+i*0.1,0);
    }*/


    //////////////////////////////////////////////////////////////////////////
    // (IN)VISIBLE WALLS AND GROUND
    //////////////////////////////////////////////////////////////////////////

    // create 5 static walls to contain the dynamic objects within a workspace
    bulletInvisibleWall1 = new cBulletStaticPlane(bulletWorld,
                           cVector3d(0.0, 0.0, -1.0), -0.1*scale); // top
    bulletInvisibleWall2 = new cBulletStaticPlane(bulletWorld,
                           cVector3d(0.0, -1.0, 0.0), -0.1*scale); // right
    bulletInvisibleWall3 = new cBulletStaticPlane(bulletWorld,
                           cVector3d(0.0, 1.0, 0.0),  -0.1*scale); // left
    bulletInvisibleWall4 = new cBulletStaticPlane(bulletWorld,
                           cVector3d(-1.0, 0.0, 0.0), -0.06*scale);// front
    bulletInvisibleWall5 = new cBulletStaticPlane(bulletWorld,
                           cVector3d(1.0, 0.0, 0.0), -0.04*scale); // back

    // create ground plane
    bulletGround = new cBulletStaticPlane(bulletWorld, cVector3d(0.0, 0.0, 1.0), -0.05);

    // add plane to world as we will want to make it visibe
    bulletWorld->addChild(bulletGround);
    namemap[bulletGround->m_bulletRigidBody] = "Floor";

    // create a mesh plane where the static plane is located
    cCreatePlane(bulletGround, 0.2*scale, 0.2*scale,
             bulletGround->getPlaneConstant() * bulletGround->getPlaneNormal());

    // define some material properties and apply to mesh
    cMaterial matGround;
    matGround.setStiffness(stiffness);
    matGround.setDynamicFriction(0.1);
    matGround.setStaticFriction(0.0);
    matGround.setWhite();
    matGround.m_emission.setGrayLevel(0.3f);
    bulletGround->setMaterial(matGround);

#ifdef TOOLS
    // setup collision detector for haptic interaction
    bulletGround->createAABBCollisionDetector(avatarRadius);

    // set friction values
    bulletGround->setSurfaceFriction(0.4);
#endif

    // Display walls too (they are invisible in the default example)
    cBulletStaticPlane* walls[] = { 0, bulletInvisibleWall1,
                                       bulletInvisibleWall2,
                                       bulletInvisibleWall3,
                                       bulletInvisibleWall4,
                                       bulletInvisibleWall5 };
    std::string wallNames[]={"","Ceiling","RightWall","LeftWall","FrontWall","BackWall"};
    for(int i=1;i<=5;i++){
        namemap[walls[i]->m_bulletRigidBody]=wallNames[i];

        bulletWorld->addChild(walls[i]);
        cMatrix3d rot;
        rot.identity();
        if(i==5) // back
            rot.rotateAboutLocalAxisDeg(0,1,0,90);
        else if(i==1)
            rot.rotateAboutLocalAxisDeg(1,0,0,180);
        else if(i==2)
            rot.rotateAboutLocalAxisDeg(1,0,0,90);
        else if(i==4)
            rot.rotateAboutLocalAxisDeg(0,1,0,-90);
        else if(i==3)
            rot.rotateAboutLocalAxisDeg(1,0,0,-90);

        if(i==2||i==3){ // sides
            cCreatePlane(walls[i],0.2*scale,0.16*scale,
                  walls[i]->getPlaneConstant()*walls[i]->getPlaneNormal(), rot);
            walls[i]->translate(-0.04*scale,0,0.02*scale);
        }
        else {
            cCreatePlane(walls[i],0.16*scale,0.2*scale,
                walls[i]->getPlaneConstant()*walls[i]->getPlaneNormal(), rot);
            if(i==5 || i==4)
                walls[i]->translate(0.0,0,0.02*scale);
        }

        matGround.setGreenLight();
        if(i==4 || i==1){
            walls[i]->setShowEnabled(false);
        }
        walls[i]->setMaterial(matGround);

        walls[i]->m_bulletRigidBody->setFriction(0.9);

#ifdef TOOLS
        // Following makes them haptic enabled
        walls[i]->createAABBCollisionDetector(avatarRadius);
        walls[i]->setSurfaceFriction(0.4);
#endif
    }



    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);



    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------
    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    std::cout << "exits main()\n";
    release(0);
    release(1);

    // exit
    return 0;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width = a_width;
    height = a_height;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}
//------------------------------------------------------------------------------




//------------------------------------------------------------------------------
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        // Better stop haptic loop before attempting to destroy
        // window etc. SOMETIMES it crashes on exit otherwise, difficult
        // to debug, although theoretically it should be enough
        // to do this in close()
        simulationRunning=false;
        while (!simulationFinished) { cSleepMs(100); }

        // Now we can close window
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - enable/disable gravity
    else if (a_key == GLFW_KEY_G)
    {
        if (bulletWorld->getGravity().length() > 0.0)
        {
            bulletWorld->setGravity(0.0, 0.0, 0.0);
        }
        else
        {
            bulletWorld->setGravity(0.0, 0.0,-9.82);
        }
    }

    // option - enable/disable gravity
    else if (a_key == GLFW_KEY_SPACE)
    {
        requestAddBox=true;
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);        

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width,
                                 mode->height, 120);// mode->refreshRate);
            int w = int(0.8 * mode->height);
            int h = int(0.5 * mode->height);
            int x = int(0.5 * (mode->width - w));
            int y = int(0.5 * (mode->height - h));
            glfwSwapInterval(swapInterval);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, 120);
            glfwSwapInterval(swapInterval);
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width,
                                 mode->height, 120);// mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = int(0.8 * mode->height);
            int h = int(0.5 * mode->height);
            int x = int(0.5 * (mode->width - w));
            int y = int(0.5 * (mode->height - h));
            glfwSetWindowMonitor(window, NULL, x, y, w, h, 120);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }


    // Pick up first device (if in contact)
    else if (a_key == GLFW_KEY_1)
    {
        if(dof6[0]) {
            requestedPickup[0]=false;
            requestedRelease[0]=true;
        } else {
            requestedPickup[0]=true;
            requestedRelease[0]=false;
        }
    }

    // Pick up second device (if in contact)
    else if (a_key == GLFW_KEY_2)
    {
        if(dof6[1]) {
            requestedPickup[1]=false;
            requestedRelease[1]=true;
        } else {
            requestedPickup[1]=true;
            requestedRelease[1]=false;
        }
    }
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void close(void)
{
    std::cout << "in close()\n";
    // stop the simulation
    requestedRelease[0]=true;
    requestedRelease[1]=true;
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
#ifdef TOOLS
    for(int i=0;i<numDevices;++i)
        tool[i]->stop();
#endif




#ifdef BULLET_FOR_AVATARS
    for(int i=0;i<2;++i){
        if(acon[i]){
            bulletWorld->m_bulletWorld->removeConstraint(acon[i]);
            delete acon[i];
        }
    }
#endif

    // delete resources
    delete hapticsThread;
    delete bulletWorld;
    delete handler;
}
//-----------------------------------------------------------------------------


//------------------------------------------------------------------------------
void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    bulletWorld->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
}
//---------------------------------------------------------------------------


//------------------------------------------------------------------------------
void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // reset clock
    cPrecisionClock clock;
    clock.reset();
    wall_clock.reset();
    wall_clock.start();

    // main haptic simulation loop
    while(simulationRunning)
    {

        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME    
        /////////////////////////////////////////////////////////////////////

        // signal frequency counter
        freqCounterHaptics.signal(1);


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        bulletWorld->computeGlobalPositions(true);

        // Get button states
        for(int t=0;t<numDevices;t++){
            hapticDevice[t]->getUserSwitch(0,currentButton[t]);

            // release other if holding
            if(!currentButton[t]) holdOther[t]=false;

            // Request release if we hold, and have released button
            if(previousButton[t] && !currentButton[t] && dof6[t])
                requestedRelease[t]=true;
        }


#ifdef BULLET_FOR_AVATARS
        cVector3d totalForce[] = {cVector3d(0,0,0), cVector3d(0,0,0)};
#endif

#ifdef TOOLS
        for(int i=0;i<numDevices;++i){
            // update position and orientation of tool
            tool[i]->updateFromDevice();

            // Show (for debug)
            deviceSphere[i]->setLocalPos(tool[i]->getDeviceGlobalPos());

            if(dof6[i]){
                cBulletBox* obj = FindChaiObject(&(dof6[i]->getRigidBodyB()));
                obj->setHapticEnabled(false);
            }

            // compute interaction forces
            tool[i]->computeInteractionForces();

            if(dof6[i]){
                cBulletBox* obj = FindChaiObject(&(dof6[i]->getRigidBodyB()));
                obj->setHapticEnabled(true);
            }

            // Compute tool-tool interaction
            cVector3d f_tool_tool = cVector3d(0,0,0);
            cGenericTool* other = i==0? tool[1] : tool[0];
            cVector3d other_pos = other->m_image->getGlobalPos();
            cVector3d my_pos = tool[i]->m_image->getGlobalPos();
            double dist = (other_pos - my_pos).length();
            cVector3d dir = other_pos - my_pos;
            dir.normalize();


            // Holding other?
            int other_n=i==0?1:0;
            if(holdOther[other_n]){ // that is: me being held
                f_tool_tool = (stiffness/scale)*(dist-2*avatarRadius*1.1) * dir;
                if(f_tool_tool.length()>5) f_tool_tool=dir*5;
            }

            if(dist < 2*avatarRadius*1.1 && dist > 0.00001){
                f_tool_tool += (stiffness/scale)*(dist-(2*avatarRadius*1.1)) * dir;
                if(currentButton[i]){
                    holdOther[i]=true;
                }
            }

            tool[i]->addDeviceGlobalForce(f_tool_tool);
        }






        /////////////////////////////////////////////////////////////////////
        // DYNAMIC SIMULATION
        /////////////////////////////////////////////////////////////////////

        // for each interaction point of the tool we look for any contact events
        // with the environment and apply forces accordingly
        for(int t=0;t<numDevices;t++){
            int numInteractionPoints = tool[t]->getNumHapticPoints();
            for (int i=0; i<numInteractionPoints; i++)
            {
                // get pointer to next interaction point of tool
                cHapticPoint* interactionPoint = tool[t]->getHapticPoint(i);

                // check all contact points
                int numContacts = interactionPoint->getNumCollisionEvents();
                for (int i=0; i<numContacts; i++)
                {
                    cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(i);

                    // given the mesh object we may be touching, we search for its owner which
                    // could be the mesh itself or a multi-mesh object. Once the owner found, we
                    // look for the parent that will point to the Bullet object itself.
                    cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

                    // cast to Bullet object
                    cBulletGenericObject* bulletobject = dynamic_cast<cBulletGenericObject*>(object);

                    // if Bullet object, we apply interaction forces
                    if (bulletobject != NULL)
                    {
                        // Check for pick-up
                        // Request pickup with button (if not holding something)
                        if(!previousButton[t] && currentButton[t] && !dof6[t])
                            requestedPickup[t]=true;

                        if(requestedPickup[t] && !dof6[t] && !holdOther[t]){
                            for(auto p : bulletBoxes)
                                if(p==bulletobject)
                                    pickup(bulletobject->m_bulletRigidBody,t);
                        }
                        else if(dof6[t] && &(dof6[t]->getRigidBodyB())==bulletobject->m_bulletRigidBody)
                            continue; // do not add forces to the object we hold.
#ifdef HAPTIC_PUSH
                        else{
                            cVector3d push = -interactionPoint->getLastComputedForce();
                            // Scale pushing to increase friction + forces when
                            // box lies on top of object
                            push = cVector3d(push.x()*hapticPushFactors.x(),
                                             push.y()*hapticPushFactors.y(),
                                             push.z()*hapticPushFactors.z());
                            bulletobject->addExternalForceAtPoint(push,
                                           collisionEvent->m_globalPos -
                                           object->getLocalPos());
                        }
#endif
                    }
                }
            }
        }
#endif

#ifdef BULLET_FOR_AVATARS
        for(int i=0;i<2;++i){
            if(acon[i]){
                cVector3d pos;
                hapticDevice[i]->getPosition(pos);
                //pos+=cVector3d(0,0,-0.07);
                deviceSphere[i]->setLocalPos(pos);
                //std::cout << "Device " << i << " pos " << pos << "\n";
                btVector3 newPivot = btVector3(pos.x(), pos.y(), pos.z());
                acon[i]->getFrameOffsetA().setOrigin(newPivot);


                double k = stiffness;
                btVector3 f  = acon[i]->getCalculatedTransformA().getOrigin() - acon[i]->getCalculatedTransformB().getOrigin();
                cVector3d cf = -k*b2c(f) + applied_f[i];
                //hapticDevice[i]->setForce(cf);
                totalForce[i] += cf;
            }
        }
#endif

        // Check for releasing object
        for(int t=0;t<numDevices;t++){
            if(requestedRelease[t]) // device button mode
                release(t);
        }


        // If we have picked up an object
        for(int t=0;t<numDevices;t++){
            if(dof6[t]){

#ifdef TOOLS
                cVector3d pos = tool[t]->m_image->getGlobalPos();
#else
                cVector3d pos = avatar[t]->getGlobalPos();
#endif
               btVector3 newA = btVector3(pos.x(), pos.y(), pos.z());

                // Check if we are pulling it too far when constrained (didnt work well that either)
                /*
                cVector3d diff = b2c(dof6[t]->getCalculatedTransformA().getOrigin() - dof6[t]->getCalculatedTransformB().getOrigin());
                cVector3d dir = diff;
                double length = diff.length();
                dir.normalize();

                if(length>0.02)
                    newA = c2b(dir*0.02);
                */


                dof6[t]->getFrameOffsetA().setOrigin(newA);

                //btVector3 f = bulletBox0->m_bulletRigidBody->getTotalForce();
                //std::cout << "force " << f.x() << " " << f.y() << " " << f.z() << "\n";
                btVector3 applied_f = dof6[t]->getRigidBodyB().getTotalForce();

                //std::cout<< "applied: " << dof6[t]->getAppliedImpulse() <<"\n";

                cVector3d capplied_f = cVector3d(applied_f.x(),applied_f.y(), applied_f.z());

                double k = stiffness/scale;
                cVector3d d = b2c(dof6[t]->getCalculatedTransformA().getOrigin() -
                                  dof6[t]->getCalculatedTransformB().getOrigin());
                cVector3d boxForce=-k*d;
                //if(boxForce.length()<0.1) boxForce.zero();

                // How many are holding?
                int numHolders=0;
                for(int tt=0;tt<numDevices;tt++)
                    if(dof6[tt] && &(dof6[tt]->getRigidBodyB()) == &(dof6[t]->getRigidBodyB()))
                        numHolders++;

                // set mass to start value since we temporarily change it in pickup()
                double mass = boxHapticMass; //1.0/dof6[t]->getRigidBodyB().getInvMass()
                cVector3d haptic_grav = cVector3d(0, 0, mass * -9.82 / double(numHolders));
                cVector3d cf = boxForce + capplied_f + haptic_grav;
                //std::cout << "cf " << t << " : "<< cf << "\n";

#ifdef TOOLS
                tool[t]->addDeviceGlobalForce(cf);
#else
                totalForce[t] += cf;
#endif
            }
        }


        // send forces to haptic devices
#ifdef TOOLS
        //std::cout << "Forces: ";
        for(int i=0;i<numDevices;++i){

            // Wait for small forces. We implement this here since
            // we want the option of adding a constant force after
            // the check and we cant read tool.m_engageForces
            if(!engageForces[i]){
                if(tool[i]->getDeviceGlobalForce().length() < smallForceThresh)
                    smallForceCounter[i]++;
                if(smallForceCounter[i]>3)
                    engageForces[i]=true;
                else
                    continue;
            }

#ifdef FORCE_AVERAGING
            tool[i]->addDeviceGlobalForce(0,0,manipulandumGravityCompensation);
            cVector3d f_to_send = tool[i]->getDeviceGlobalForce();
            if(f_to_send.length() > prevForce[i].length()+0.1){
                f_to_send = (f_to_send+prevForce[i])*0.5;
            }
            prevForce[i]=f_to_send;
            tool[i]->setDeviceGlobalForce(f_to_send);
#endif
            if(!disable_haptics[i])
                tool[i]->applyToDevice();
            //printNice(tool[i]->getDeviceGlobalForce());
        }
        //std::cout << "\n";
#endif

#ifdef BULLET_FOR_AVATARS
        std::cout << "set force: " << std::setprecision(3) << std::fixed;
        for(int i=0;i<2;++i){
            if(!disable_haptics[i])
            hapticDevice[i]->setForce(totalForce[i]);
            std::cout << "      " << std::setw(6) << totalForce[i].x() << ", "
                                  << std::setw(6) << totalForce[i].y() << ", "
                                  << std::setw(6) << totalForce[i].z();
        }
        std::cout << "\n";
#endif


        // Check for Bullet collisions and throw events
        CheckForCollisionEvents();

        // Process time-based events
        tick_wall_clock_events();

        // stop the bullet dynamics simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = cClamp(clock.getCurrentTimeSeconds(), 0.0001, 0.001);

        // restart the simulation clock
        clock.reset();
        clock.start();

        // Move our gravity
#ifdef MY_GRAVITY
        for(auto b:bulletBoxes){
            if(gravityGo[b]){
                gravityVelocity[b]=-1.0;
            } else
                gravityVelocity[b]=0.0;

            unfreeze(b->m_bulletRigidBody);
            bulletWorld->computeGlobalPositions(true);
            b->m_bulletRigidBody->translate(btVector3(0,0,-0.01));
            bulletWorld->computeGlobalPositions(true);
            freeze(b->m_bulletRigidBody);
        }
#endif


        if(requestAddBox){
            requestAddBox=false;
            addBox();
        }

        if(!simulationRunning){
            release(0);
            release(1);
        }

        // update simulation
        bulletWorld->updateDynamics(timeInterval);


        // Send info to OSC
#ifdef OSC
        // Send OSC data
        for(int t=0;t<numDevices;++t){
#ifdef TOOLS
            cVector3d currentForce = tool[t]->getDeviceGlobalForce();
            cVector3d currentPos = tool[t]->m_image->getGlobalPos();
            cVector3d currentVel = tool[t]->getDeviceGlobalLinVel();
#endif
            char buffer[OUTPUT_BUFFER_SIZE];
            osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);
            p.Clear();
            string addr = t==0?"/device0":"/device1";
            p << osc::BeginMessage(addr.c_str())
              << float(currentPos.x()) << float(currentPos.y()) << float(currentPos.z())
              << float(currentForce.x()) << float(currentForce.y()) << float(currentForce.z())
              << float(currentVel.x()) << float(currentVel.y()) << float(currentVel.z())
              << int(holdOther[t])
              << osc::EndMessage;
            transmitSocket->Send( p.Data(), p.Size() );
        }
#endif

        for(int t=0;t<numDevices;++t)
            previousButton[t]=currentButton[t];

    }

    // exit haptics thread
    simulationFinished = true;
}
//------------------------------------------------------------------------------
