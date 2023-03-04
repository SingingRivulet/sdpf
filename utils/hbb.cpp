#include "hbb.h"
#include "mempool.h"
namespace sdpf {

typedef mempool<HBB::boundCircle> apool;

void HBB::boundCircle::autoclean() {
    if (left == NULL && right == NULL && !isDataNode()) {
        if (parent) {
            if (parent->left == this) {
                parent->left = NULL;
            }
            if (parent->right == this) {
                parent->right = NULL;
            }
            parent->autoclean();
            hbb->delAABB(this);
        }
    } else if (parent && parent->parent) {
        if (parent->left && parent->right == NULL)
            parent->left = NULL;
        else if (parent->left == NULL && parent->right)
            parent->right = NULL;
        else
            return;

        if (parent->parent->left == parent) {
            parent->parent->left = this;
        } else {
            parent->parent->right = this;
        }

        auto tmp = parent;
        parent = parent->parent;
        hbb->delAABB(tmp);
        parent->autoclean();
    }
}

void HBB::boundCircle::add(boundCircle* in) {
    if (left) {
        if (!left->isDataNode() && in->inBox(left)) {
            left->add(in);
            return;
        } else if (right == NULL) {
            setRight(in);
            return;
        }
    }
    if (right) {
        if (!right->isDataNode() && in->inBox(right)) {
            right->add(in);
            return;
        } else if (left == NULL) {
            setLeft(in);
            return;
        }
    }
    if (right == NULL && left == NULL) {
        setLeft(in);
        return;
    }

    auto ls = left->getMergeSizeSq(in);
    auto rs = right->getMergeSizeSq(in);
    auto nnode = hbb->createAABB();

    //nnode->parent=this;

    if (ls < rs) {
        in->merge(left, nnode);
        nnode->setLeft(left);
        nnode->setRight(in);
        this->setLeft(nnode);
    } else {
        in->merge(right, nnode);
        nnode->setLeft(right);
        nnode->setRight(in);
        this->setRight(nnode);
    }
    //debug
    //printf(
    //    "create:(%f,%f,%f)->(%f,%f,%f)\n",
    //    nnode->from.x,nnode->from.y,nnode->from.Z,
    //    nnode->to.x,nnode->to.y,nnode->to.Z
    //);
}

void HBB::poolInit() {
    pool = new apool;
}
void HBB::poolDestroy() {
    if (pool)
        delete (apool*)pool;
}

HBB::boundCircle* HBB::createAABB() {
    if (pool) {
        auto p = ((apool*)pool)->get();
        p->construct();
        p->hbb = this;
        return p;
    }
    return NULL;
}
void HBB::delAABB(HBB::boundCircle* p) {
    if (pool) {
        ((apool*)pool)->del(p);
    }
}

void HBB::boundCircle::remove() {
    if (parent) {
        if (parent->left == this) {
            parent->left = NULL;
        }
        if (parent->right == this) {
            parent->right = NULL;
        }
        parent->autoclean();
        parent = NULL;
    }
}

void HBB::boundCircle::drop() {
    if (left) {
        left->drop();
        left = NULL;
    }
    if (right) {
        right->drop();
        right = NULL;
    }
    if (parent) {
        if (parent->left == this) {
            parent->left = NULL;
        }
        if (parent->right == this) {
            parent->right = NULL;
        }
        parent = NULL;
    }
    hbb->delAABB(this);
}

void HBB::boundCircle::autodrop() {
    auto p = parent;
    this->drop();
    if (p)
        p->autoclean();
}

void HBB::add(HBB::boundCircle* in) {
    root->add(in);
}

void HBB::remove(HBB::boundCircle* in) {
    in->remove();
}

HBB::boundCircle* HBB::add(const vec& center, double r, void* data) {
    auto p = createAABB();
    p->center = center;
    p->r = r;
    p->data = data;
    root->add(p);
    return p;
}

void HBB::boundCircle::collisionTest(
    const boundCircle* in,
    void (*callback)(boundCircle*, void*),
    void* arg) {
    if (left && left->intersects(in)) {
        if (left->isDataNode())
            callback(left, arg);
        else
            left->collisionTest(in, callback, arg);
    }

    if (right && right->intersects(in)) {
        if (right->isDataNode())
            callback(right, arg);
        else
            right->collisionTest(in, callback, arg);
    }
}
void HBB::boundCircle::fetchByPoint(
    const vec& point,
    void (*callback)(boundCircle*, void*),
    void* arg) {
    if (left && left->inBox(point)) {
        if (left->isDataNode())
            callback(left, arg);
        else
            left->fetchByPoint(point, callback, arg);
    }
    if (right && right->inBox(point)) {
        if (right->isDataNode())
            callback(right, arg);
        else
            right->fetchByPoint(point, callback, arg);
    }
}

void HBB::boundCircle::fetchByRay(
    const vec& p0,
    const vec& p1,
    void (*callback)(boundCircle*, void*),
    void* arg) {
    if (left && left->intersects(p0, p1)) {
        if (left->isDataNode())
            callback(left, arg);
        else
            left->fetchByRay(p0, p1, callback, arg);
    }
    if (right && right->intersects(p0, p1)) {
        if (right->isDataNode())
            callback(right, arg);
        else
            right->fetchByRay(p0, p1, callback, arg);
    }
}

HBB::HBB() {
    poolInit();
    root = createAABB();
}

HBB::~HBB() {
    if (root)
        root->drop();
}

}  // namespace sdpf
