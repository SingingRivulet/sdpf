#pragma once
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
namespace sdpf::kdtree {

// kd树的节点定义
struct node {
    node* parent;
    node* leftChild;
    node* rightChild;
    std::vector<double> val;  //存储的数据
    int axis;                 // 使用的轴
    node(std::vector<double> data, int ax) {
        val = data;
        axis = ax;
        parent = nullptr;
        leftChild = nullptr;
        rightChild = nullptr;
    }
};

inline std::ostream& operator<<(std::ostream& os, std::vector<double> vi) {
    os << "(";
    for (int i = 0; i < vi.size(); i++)
        std::cout << vi[i] << ",";
    os << ")";
    return os;
}

// 导入数据，第一行是数据数量和维度，后面跟着num行dim维度的数据，每个一行
inline bool loadData(std::string filename, std::vector<std::vector<double> >& data) {
    std::ifstream infs(filename);
    if (infs.is_open()) {
        int num, dim;
        infs >> num >> dim;
        std::vector<double> d(dim);
        for (int i = 0; i < num; i++) {
            for (int j = 0; j < dim; j++)
                infs >> d[j];
            data.push_back(d);
        }
        return true;
    }
    return false;
}

// 计算N维向量距离
inline double disVector(std::vector<double> a, std::vector<double> b) {
    double sum = 0;
    for (int i = 0; i < a.size(); i++)
        sum += (a[i] - b[i]) * (a[i] - b[i]);
    return sum;
}

class tree {
   private:
    int dimension;
    std::vector<std::vector<double> > data;
    node* root;
    std::vector<node*> nodes;

   public:
    inline tree(std::vector<std::vector<double> > d, int dim) {
        dimension = dim;
        data = d;
        createTree();
    }
    inline ~tree() {
        for (auto it : nodes) {
            delete it;
        }
        nodes.clear();
    }
    inline void createTree() {
        // 递归建树
        root = createTreeNode(0, data.size() - 1, 0);
    }

    // create Kd Tree struct
    inline node* createTreeNode(int left, int right, int dim) {
        if (right < left)
            return nullptr;
        // 按照k维进行排序
        std::sort(data.begin() + left, data.begin() + right + 1,
                  [&](std::vector<double> a, std::vector<double> b) {
                      if (a[dim] < b[dim])
                          return true;
                      return false;
                  });
        int mid = (left + right + 1) / 2;
        node* r = new node(data[mid], dim);
        nodes.push_back(r);
        r->leftChild = createTreeNode(left, mid - 1, (dim + 1) % dimension);
        if (r->leftChild != nullptr)
            r->leftChild->parent = r;
        r->rightChild = createTreeNode(mid + 1, right, (dim + 1) % dimension);
        if (r->rightChild != nullptr)
            r->rightChild->parent = r;
        return r;
    }

    inline void printKdTree() {
        printKdTreeNode(root);
    }

    inline void printKdTreeNode(node* r) {
        if (r == nullptr)
            return;
        printKdTreeNode(r->leftChild);
        std::cout << r->val << "\t";
        printKdTreeNode(r->rightChild);
        std::cout << std::endl;
    }

    // 查找kd树
    inline node* searchKdTree(std::vector<double> d) {
        int dim = 0;
        double minDis = 10000000;
        node* r = root;
        node* tmp;
        while (r != nullptr) {
            tmp = r;
            if (d[dim] < r->val[dim])
                r = r->leftChild;
            else
                r = r->rightChild;
            dim = (dim + 1) % dimension;
        }
        // 找到属于的那个节点
        minDis = std::min(disVector(d, tmp->val), minDis);
        node* nearNode = tmp;
        //std::cout << std::endl
        //          << "nearest node: " << tmp->val << std::endl;
        // 退回到根节点
        while (tmp->parent != nullptr) {
            tmp = tmp->parent;
            // 判断父节点是否更近，如果近，更新最近节点
            if (disVector(tmp->val, d) < minDis) {
                nearNode = tmp;
                minDis = disVector(tmp->val, d);
            }
            //std::cout << "now parent node: " << tmp->val << std::endl;
            node* son;
            // 判断当前轴与点的距离，如果小于minDis，则进行到另一半进行查找
            if (abs(tmp->val[tmp->axis] - d[tmp->axis]) < minDis) {
                if (tmp->val[tmp->axis] > d[tmp->axis])
                    son = tmp->rightChild;
                else
                    son = tmp->leftChild;
                searchKdTreeNode(d, minDis, nearNode, son);
            }
        }
        // 对根节点的另外半边子树进行搜索
        /*if (abs(tmp->val[tmp->axis] - d[tmp->axis]) < minDis)
		{
			if (tmp->val[tmp->axis] > d[tmp->axis])
				tmp = tmp->rightChild;
			else
				tmp = tmp->leftChild;
			searchKdTreeNode(d, minDis, nearNode, tmp);
		}*/
        return nearNode;
    }

    // 查找当前节点下的最近点
    inline void searchKdTreeNode(std::vector<double> d, double& minDis, node*& nearNode, node* tmp) {
        // 递归终止
        if (tmp == nullptr)
            return;
        //std::cout << "now node: " << tmp->val << std::endl;
        // 判断当前节点是否小于
        if (disVector(tmp->val, d) < minDis) {
            minDis = disVector(tmp->val, d);
            nearNode = tmp;
        }
        // 如果轴与节点的距离小于minDis，则两个半边都需要搜索，否则只需要搜索一个半边
        if (abs(tmp->val[tmp->axis] - d[tmp->axis]) < minDis) {
            searchKdTreeNode(d, minDis, nearNode, tmp->leftChild);
            searchKdTreeNode(d, minDis, nearNode, tmp->rightChild);
        } else {
            // 选择搜索的一个半边
            if (tmp->val[tmp->axis] > d[tmp->axis])
                searchKdTreeNode(d, minDis, nearNode, tmp->leftChild);
            else
                searchKdTreeNode(d, minDis, nearNode, tmp->rightChild);
        }
    }
};

}  // namespace sdpf::kdtree