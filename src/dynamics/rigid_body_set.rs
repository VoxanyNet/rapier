use diff::{Diff, VecDiff};

use crate::data::arena::ArenaDiff;
use crate::data::{Arena, Index};
use crate::dynamics::{
    ImpulseJointSet, IslandManager, MultibodyJointSet, RigidBody, RigidBodyChanges, RigidBodyHandle,
};
use crate::geometry::ColliderSet;
use crate::prelude::{ColliderHandle};
use core::sync;
use std::collections::HashMap;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A pair of rigid body handles.
pub struct BodyPair {
    /// The first rigid body handle.
    pub body1: RigidBodyHandle,
    /// The second rigid body handle.
    pub body2: RigidBodyHandle,
}

impl BodyPair {
    /// Builds a new pair of rigid-body handles.
    pub fn new(body1: RigidBodyHandle, body2: RigidBodyHandle) -> Self {
        BodyPair { body1, body2 }
    }
}


#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Default, Debug, PartialEq)]
/// A set of rigid bodies that can be handled by a physics pipeline.
pub struct RigidBodySet {
    // NOTE: the pub(crate) are needed by the broad phase
    // to avoid borrowing issues. It is also needed for
    // parallelism because the `Receiver` breaks the Sync impl.
    // Could we avoid this?
    pub bodies: Arena<RigidBody>,
    pub(crate) modified_bodies: Vec<RigidBodyHandle>, // do we need to diff this?
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct RigidBodySetDiff {
    pub bodies: Option<ArenaDiff<RigidBody>>,
    pub modified_bodies: Option<VecDiff<RigidBodyHandle>>
}

impl Diff for RigidBodySet {
    type Repr = RigidBodySetDiff;
    
    fn diff(&self, other: &Self) -> Self::Repr {

        let mut diff = RigidBodySetDiff {
            bodies: None,
            modified_bodies: None
        };

        if other.bodies != self.bodies {

            diff.bodies = Some(self.bodies.diff(&other.bodies));

        };

        if other.modified_bodies != self.modified_bodies {
            diff.modified_bodies = Some(self.modified_bodies.diff(&other.modified_bodies))
        }

        diff
    }
    
    fn apply(&mut self, diff: &mut Self::Repr) {
        if let Some(bodies_diff) = &mut diff.bodies {
            self.bodies.apply(bodies_diff);
        }

        if let Some(modified_bodies_diff) = &mut diff.modified_bodies {
            self.modified_bodies.apply(modified_bodies_diff);
        }
    }
    
    fn identity() -> Self {
        Self::default()
    } 

    
}

impl RigidBodySet {
    /// Create a new empty set of rigid bodies.
    pub fn new() -> Self {
        RigidBodySet {
            bodies: Arena::new(),
            modified_bodies: Vec::new(),
        }
    }

    pub(crate) fn take_modified(&mut self) -> Vec<RigidBodyHandle> {

        
        std::mem::take(&mut self.modified_bodies)
    }

    pub fn get_local_index(&self, sync_id: u64) -> Option<RigidBodyHandle> {
        match self.bodies.sync_index_map.get(&sync_id) {
            Some((local_index, local_generation)) => {
                Some(RigidBodyHandle(Index::from_raw_parts(*local_index, *local_generation, sync_id)))
            },
            None => None,
        }
    }

    pub fn get_sync(&self, sync_id: u64) -> Option<&RigidBody> {
        match self.bodies.sync_index_map.get(&sync_id) {
            Some((rigid_body_index, rigid_body_generation)) => {
                self.bodies.get(&mut Index::from_raw_parts(*rigid_body_index, *rigid_body_generation, sync_id))  
            },
            None => None,
        }
    }

    pub fn get_sync_mut(&mut self, sync_id: u64) -> Option<&mut RigidBody> {
        match self.bodies.sync_index_map.get(&sync_id) {
            Some((rigid_body_index, rigid_body_generation)) => {
                self.bodies.get_mut(&mut Index::from_raw_parts(*rigid_body_index, *rigid_body_generation, sync_id)) 
            },
            None => None,
        }
    }

    /// The number of rigid bodies on this set.
    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    /// `true` if there are no rigid bodies in this set.
    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty()
    }

    /// Is the given body handle valid?
    pub fn contains(&self, handle: &mut RigidBodyHandle) -> bool {
        self.bodies.contains(&mut handle.0)
    }

    /// Insert a rigid body into this set and retrieve its handle.
    pub fn insert(&mut self, rb: impl Into<RigidBody>) -> RigidBodyHandle {
        let mut rb = rb.into();
        // Make sure the internal links are reset, they may not be
        // if this rigid-body was obtained by cloning another one.
        rb.reset_internal_references();
        rb.changes.set(RigidBodyChanges::all(), true);

        let handle = RigidBodyHandle(self.bodies.insert(rb));
        self.modified_bodies.push(handle);
        handle
    }

    /// Removes a rigid-body, and all its attached colliders and impulse_joints, from these sets.
    pub fn remove(
        &mut self,
        handle: RigidBodyHandle,
        islands: &mut IslandManager,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
        remove_attached_colliders: bool,
    ) -> Option<RigidBody> {
        let mut rb = self.bodies.remove(handle.0)?;
        /*
         * Update active sets.
         */
        islands.rigid_body_removed(handle, &rb.ids, self);

        /*
         * Remove colliders attached to this rigid-body.
         */
        if remove_attached_colliders {
            for collider in rb.colliders() {
                colliders.remove(*collider, islands, self, false);
            }
        } else {
            // If we don’t remove the attached colliders, simply detach them.
            let colliders_to_detach = rb.colliders().to_vec();
            for co_handle in colliders_to_detach {
                colliders.set_parent(co_handle, None, self);
            }
        }

        /*
         * Remove impulse_joints attached to this rigid-body.
         */
        impulse_joints.remove_joints_attached_to_rigid_body(handle);
        multibody_joints.remove_joints_attached_to_rigid_body(handle);

        Some(rb)
    }

    /// Gets the rigid-body with the given handle without a known generation.
    ///
    /// This is useful when you know you want the rigid-body at position `i` but
    /// don't know what is its current generation number. Generation numbers are
    /// used to protect from the ABA problem because the rigid-body position `i`
    /// are recycled between two insertion and a removal.
    ///
    /// Using this is discouraged in favor of `self.get(handle)` which does not
    /// suffer form the ABA problem.
    pub fn get_unknown_gen(&self, i: u32) -> Option<(&RigidBody, RigidBodyHandle)> {
        self.bodies
            .get_unknown_gen(i)
            .map(|(b, h)| (b, RigidBodyHandle(h)))
    }

    /// Gets a mutable reference to the rigid-body with the given handle without a known generation.
    ///
    /// This is useful when you know you want the rigid-body at position `i` but
    /// don't know what is its current generation number. Generation numbers are
    /// used to protect from the ABA problem because the rigid-body position `i`
    /// are recycled between two insertion and a removal.
    ///
    /// Using this is discouraged in favor of `self.get_mut(handle)` which does not
    /// suffer form the ABA problem.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn get_unknown_gen_mut(&mut self, i: u32) -> Option<(&mut RigidBody, RigidBodyHandle)> {
        let (rb, handle) = self.bodies.get_unknown_gen_mut(i)?;
        let handle = RigidBodyHandle(handle);
        Self::mark_as_modified(handle, rb, &mut self.modified_bodies);
        Some((rb, handle))
    }

    /// Gets the rigid-body with the given handle.
    pub fn get(&self, handle: RigidBodyHandle) -> Option<&RigidBody> {
        self.bodies.get(&mut handle.0.clone())
    }

    pub(crate) fn mark_as_modified(
        handle: RigidBodyHandle,
        rb: &mut RigidBody,
        modified_bodies: &mut Vec<RigidBodyHandle>,
    ) {
        if !rb.changes.contains(RigidBodyChanges::MODIFIED) {
            rb.changes = RigidBodyChanges::MODIFIED;
            modified_bodies.push(handle);
        }
    }

    /// Gets a mutable reference to the rigid-body with the given handle.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn get_mut(&mut self, handle: &mut RigidBodyHandle) -> Option<&mut RigidBody> {
        let result = self.bodies.get_mut(&mut handle.0)?;
        Self::mark_as_modified(*handle, result, &mut self.modified_bodies);
        Some(result)
    }

    pub(crate) fn get_mut_internal(&mut self, handle: RigidBodyHandle) -> Option<&mut RigidBody> {
        self.bodies.get_mut(&mut handle.0.clone())
    }

    pub(crate) fn index_mut_internal(&mut self, handle: RigidBodyHandle) -> &mut RigidBody {
        &mut self.bodies[&mut handle.0.clone()]
    }

    // Just a very long name instead of `.get_mut` to make sure
    // this is really the method we wanted to use instead of `get_mut_internal`.
    pub(crate) fn get_mut_internal_with_modification_tracking(
        &mut self,
        handle: &mut RigidBodyHandle,
    ) -> Option<&mut RigidBody> {
        let result = self.bodies.get_mut(&mut handle.0)?;
        Self::mark_as_modified(*handle, result, &mut self.modified_bodies);
        Some(result)
    }

    /// Iterates through all the rigid-bodies on this set.
    pub fn iter(&self) -> impl Iterator<Item = (RigidBodyHandle, &RigidBody)> {
        self.bodies.iter().map(|(h, b)| (RigidBodyHandle(h), b))
    }

    /// Iterates mutably through all the rigid-bodies on this set.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (RigidBodyHandle, &mut RigidBody)> {
        self.modified_bodies.clear();
        let modified_bodies = &mut self.modified_bodies;
        self.bodies.iter_mut().map(move |(h, b)| {
            modified_bodies.push(RigidBodyHandle(h));
            (RigidBodyHandle(h), b)
        })
    }

    /// Update colliders positions after rigid-bodies moved.
    ///
    /// When a rigid-body moves, the positions of the colliders attached to it need to be updated.
    /// This update is generally automatically done at the beginning and the end of each simulation
    /// step with `PhysicsPipeline::step`. If the positions need to be updated without running a
    /// simulation step (for example when using the `QueryPipeline` alone), this method can be called
    /// manually.  
    pub fn propagate_modified_body_positions_to_colliders(&self, colliders: &mut ColliderSet) {
        for body in self.modified_bodies.iter().filter_map(|h| self.get(*h)) {
            if body.changes.contains(RigidBodyChanges::POSITION) {
                for handle in body.colliders() {
                    if let Some(collider) = colliders.get_mut(&mut handle.clone()) {
                        let new_pos = body.position() * collider.position_wrt_parent().unwrap();
                        collider.set_position(new_pos);
                    }
                }
            }
        }
    }
}

impl std::ops::Index<RigidBodyHandle> for RigidBodySet {
    type Output = RigidBody;

    fn index(&self, index: RigidBodyHandle) -> &RigidBody {
        &self.bodies[&mut index.0.clone()]
    }
}

impl std::ops::Index<crate::data::Index> for RigidBodySet {
    type Output = RigidBody;

    fn index(&self, index: crate::data::Index) -> &RigidBody {
        &self.bodies[&mut index.clone()]
    }
}

#[cfg(not(feature = "dev-remove-slow-accessors"))]
impl std::ops::IndexMut<RigidBodyHandle> for RigidBodySet {
    fn index_mut(&mut self, handle: RigidBodyHandle) -> &mut RigidBody {
        let rb = &mut self.bodies[&mut handle.0.clone()];
        Self::mark_as_modified(handle, rb, &mut self.modified_bodies);
        rb
    }
}
