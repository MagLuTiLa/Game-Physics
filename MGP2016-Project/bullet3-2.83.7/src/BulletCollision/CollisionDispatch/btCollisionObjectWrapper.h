#ifndef BT_COLLISION_OBJECT_WRAPPER_H
#define BT_COLLISION_OBJECT_WRAPPER_H

///btCollisionObjectWrapperis an internal data structure. 
///Most users can ignore this and use btCollisionObject and btCollisionShape instead
class btCollisionShape;
class btCollisionObject;
class btTransform;
#include "LinearMath/btScalar.h" // for SIMD_FORCE_INLINE definition

#define BT_DECLARE_STACK_ONLY_OBJECT \
	private: \
		void* operator new(size_t size); \
		void operator delete(void*);

struct btCollisionObjectWrapper;
struct btCollisionObjectWrapper
{
BT_DECLARE_STACK_ONLY_OBJECT

private:
	btCollisionObjectWrapper(btCollisionObjectWrapper&); // not implemented. Not allowed.
	btCollisionObjectWrapper* operator=(btCollisionObjectWrapper&);

public:
	btCollisionObjectWrapper* m_parent;
	const btCollisionShape* m_shape;
	btCollisionObject* m_collisionObject;
	const btTransform& m_worldTransform;
	int		m_partId;
	int		m_index;

	btCollisionObjectWrapper(btCollisionObjectWrapper* parent, const btCollisionShape* shape, btCollisionObject* collisionObject, const btTransform& worldTransform, int partId, int index)
	: m_parent(parent), m_shape(shape), m_collisionObject(collisionObject), m_worldTransform(worldTransform),
	m_partId(partId), m_index(index)
	{}

	SIMD_FORCE_INLINE const btTransform& getWorldTransform() const { return m_worldTransform; }
	SIMD_FORCE_INLINE  btCollisionObject* getCollisionObject()  { return m_collisionObject; }
	SIMD_FORCE_INLINE const btCollisionShape* getCollisionShape() const { return m_shape; }
};

#endif //BT_COLLISION_OBJECT_WRAPPER_H
