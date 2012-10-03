#ifndef IMP_PROTEINCHAIN_H
#define IMP_PROTEINCHAIN_H

#include "DOFVector.h"
#include "CollisionDetector.h"
#include "RotatableAngle.h"
#include "RigidPart.h"
// TODO needs a mechanism to notify the joint if the particles in it have changed position
// TODO: what can be moved to Model fast indexing system 
// TODO: check for cycles
// TODO: at first step, verify that all rigid bodies are not RigidMembers (because that would be a mess). We can extend it in the future.

// a rigid body that upstream from a joint
class KinematicNode : IMP::RigidBody{

  IMP_DECORATOR(KinematicNode, RigidBody);
        
private:
  static ParticlesKey get_children_joints_key() 
    {
        static ObjectsKey k("children_joints");
        return k;
    }

    
    static ParticleKey get_parent_joint_key() 
    {
        static ObjectKey k("parent_joint");
        return k;
    }
    
public:
    static Joint setup_particle(Particle*p, Joint parent_joint)  {
        setup_particle(p, parent_joint, Joints());
    }
    
    static Joint setup_particle(Particle*p, Joint parent_joint, Joints children_joints)  {
      if (!RigidBody::particle_is_instance(p)) {
          RigidBody::setup_particle(p, ParticlesTemp() );
      }
      p->add_attribute(get_parent_joint_key(), parent_joint)
      p->add_attribute(get_children_joints_key(), children_joints);
    }
    
    Joint* get_parent_joint() {
        // TODO: indexes system?
        Object* o = get_particle()->get_value( get_parent_joint_key() );
        return static_cast<Joint*>(o);
    }
    
    Joints get_children_joints() {
        // TODO: indexes system?
        Objects objs = get_particle()->get_value( get_children_joints_key() );
        Joints joints;
        for(unsigned int i = 0; i <= objs.size(); i++){
            Joint* j = static_cast<Joint*>(objs[i]);
            joints.push_back(j);
        }
        return joints; 
    }

    // returns the transformation that should be applied to all rigid bodies 
    // downstream of the joint. 
    // Implementation note: This may require an update based on the Cartesian
    // coordinates of the particles used to construct the joint, if those have changed
   IMP::algebra::Transformation3D get_transformation() {
        if (RevoluteJoint::particle_is_instance(get_particle()) {
            //
        }
    }

     void update_downstream_rigid_bodies(){
           algebra::Transformation3D rb_cur_trans
             =RigidBody(get_model(),
                       members[i]).get_transformation();
	   IMP::core::transform(*((RigidBody*)this), get_transformation());
//	   if(
    }

    // updates the transformation stored in the joint based on the Cartesian coordinates
    // of the appropriate particles that are stored in it to 'witness' the joint transformation
    void update_downstream_joints_from_cartesian_witnesses() {
         if (RevoluteJoint::particle_is_instance(get_particle()) {
                RevoluteJoint(get_particle()).update_joint_from_cartesian_witnesses();
         }
        // XXX and so on... XXX
    }
};

// ****************************************************************************          
class Joint : public Object{
public:
    Joint(KinematicNode from, KinematicNode to) :
        from_(from), to_(to)
    {
     // compute transformation from from_rb to to_rb
    }

    // returns the transformation that should be applied to all rigid bodies 
    // downstream of the joint
    virtual IMP::algebra::Transformation3D get_transformation() const
    { return transformation_}
    
    virtual void update_joint_from_cartesian_witnesses();   
    
    void set_transformation(IMP::algebra::Transformation3D t)
    { transformation_ = t; }

private:
    KinematicNode from_;
    KinematicNode to_;
    Transformation3D transformation_;
};

class RevoluteJoint : public Joint{
    // constructs a revolute joint on the line connecting the reference frames of
    // from_rb and to_rb    DihedralAngleRevoluteJoint(IMP::atom::RigidBody from_rb, IMP::atom::RigidBody to_rb);
    
    
    static RevoluteJoint setup_particle(Particle*p, RigidBody rb, Vector3D offset,
                                        Vector3D axis, double angle) {
        add offset, axis, angle to p
        ss=new RevoluteJointScoreState(p, rb)
        p->get_model()->add_score_state(ss)
        store score state in Particle
        add p to rb as the associated joint
        return RevoluteJoint(p)
    }
    virtual Transformation3D get_transformation() const;
    
    virtual void update_joint_from_cartesian_witnesses();
    
    void set_angle(double angle);
    
    double get_angle() const;
}

class DihedralAngleRevoluteJoint : public RevoluteJoint{
    // constructs a dihedral angle based on the last two particles in rb1 and the first two particles rb2
    // TODO: atom is not the place for dihedral, either algebra, or kinematics module
    DihedralAngleRevoluteJoint(IMP::atom::Dihedral d);
    
    virtual Transformation3D get_transformation() const;

    
    virtual void update_joint_from_cartesian_witnesses();

    void set_angle(double angle);
    
    double get_angle() const;
}

class BondAngleRevoluteJoint : public RevoluteJoint{
    // rotation of rigid body around the plane defined by three atoms
    // TODO: atom is not the place for Angle, either algebra, or kinematics module
    BondAngleRevoluteJoint(IMP::atom::Angle a);

    virtual Transformation3D get_transformation() const;
    
    virtual void update_joint_from_cartesian_witnesses();

    void set_angle(double angle);
    
    double get_angle();
private:
    IMP::atom::Angle a;
}

// joint in which too rigid bodies may slide along a line
class PrismaticJoint : public Joint{
    // TODO create prismatic decorator, probably in algebra or kinematics
    PrismaticJoint(IMP::XXX::Prismatic p);
    
    virtual Transformation3D get_transformation() const;
    
    virtual void update_joint_from_cartesian_witnesses() = 0;

    set_length(double l);
    
    double get_length() const;
private:

}

class CompositeJoint : public Joint{
    void add_joint(Joint j);
    
    virtual Transformation3D get_transformation() const;

    virtual void update_joint_from_cartesian_witnesses() = 0;
    
    Joints& get_inner_joints();
private:
    Joints joints;
}
             
             
/************************************************/             
        
class KinematicTree : public CollisionDetector { // TODO: perhaps separate collision detection capabilities?
public:
    KinematicTree(Model* m);
    
    /**
        Builds a kinematic tree automatically from a hierarchy that contains rigid bodies
        and adds a ScoreState to the model, to make sure the internal and external coordinates are synced before model updating
       TODO: think about what foldtree scheme to use (star?), 
     
     */
    KinematicTree(Model* m, IMP::Hierarchy hierarchy);
        
    void add_root_rigid_body(IMP::RigidBody rb){
        KinematicNode n = KinematicNode::setup_particle(p, NULL); 
    }
    
    // NOTE: must have root on first call
    void add_rigid_body(IMP::RigidBody parent_rb, IMP::RigidBody rb)
    {
        add_rigid_body( rb, parent_rb, TransformationJoint(rb, parent_rb) );
    }
    
    void add_rigid_body_with_dihedral(IMP::RigidBody rb, IMP::RigidBody parent_rb, 
                                      Joint joint);
    
    // first will be root? use default TransformationJoint
    add_rigid_bodies_in_chain(IMP::RigidBodies rbs);
    
    // first will be root? 
    add_rigid_bodies_in_chain(IMP::RigidBodies rbs, Joints joints);
    
    // rebuild tree (same topology but change directionality)
    reset_root(IMP::Particle* new_root); 
    
    void set_joint_to_rigid_body(IMP::Particle* rb, Joint joint);
    
    Joint& get_joint_to_rigid_body(IMP::RigidBody rb) const;

    TransformationDOF get_transformation_to_rigid_body
    (IMP::RigidBody rb) const{
        // get joint and call joint::get_transformation()
    }
   
    // returns the coordinate of any particle in the hierarchy
    // that has a rigid-body ancestor in the kinematic chain
    Vector3D get_coordinates(IMP::Particle p) const;
    
private:
    IMP::Particles get_children(IMP::Particle parent) const;
                       
    IMP::Particle get_parent(IMP::Particle child) const;
    
    friend std::ostream& operator<<(std::ostream& s, const KinematicChain& kc);
private:
    Model* m;
    bool is_cartesian_updated_;
    bool is_internal_updated_;
    <XXX tree structure XXX> tree_;
    };

#endif /* IMP_PROTEINCHAIN_H */
