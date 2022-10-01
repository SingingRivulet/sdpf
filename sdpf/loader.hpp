#pragma once
#include <sys/stat.h>
#include "navmesh.hpp"
//加载/保存
namespace sdpf::loader {

template <typename T>
inline void saveMap(field<T>& map, const std::string& path) {
    auto len = sizeof(T) * map.width * map.height;
    auto fp = fopen(path.c_str(), "wb");
    if (fp) {
        fwrite(map.data, len, 1, fp);
        fclose(fp);
    }
}

template <typename T>
inline void loadMap(field<T>& map, const std::string& path) {
    auto len = sizeof(T) * map.width * map.height;
    auto fp = fopen(path.c_str(), "rb");
    if (fp) {
        fread(map.data, len, 1, fp);
        fclose(fp);
    }
}

inline void save(navmesh::navmesh& mesh, const std::string& path) {
    auto path_config = path + "/config.txt";
    auto path_ways = path + "/ways.txt";
    auto path_nodes = path + "/nodes.txt";
    auto path_vsdfMap = path + "/vsdfMap.chunk";
    auto path_sdfMap = path + "/sdfMap.chunk";
    auto path_pathDisMap = path + "/pathDisMap.chunk";
    auto path_pathNavMap = path + "/pathNavMap.chunk";
    auto path_idMap = path + "/idMap.chunk";
    mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
    saveMap(mesh.vsdfMap, path_vsdfMap);
    saveMap(mesh.sdfMap, path_sdfMap);
    saveMap(mesh.pathDisMap, path_pathDisMap);
    saveMap(mesh.pathNavMap, path_pathNavMap);
    saveMap(mesh.idMap, path_idMap);
    {
        auto fp = fopen(path_config.c_str(), "w");
        if (fp) {
            fprintf(fp, "%d %d %lf", mesh.width, mesh.height, mesh.minItemSize);
            fclose(fp);
        }
    }
    {
        auto fp = fopen(path_ways.c_str(), "w");
        if (fp) {
            for (auto& it : mesh.ways) {
                fprintf(fp, "c%d %d %lf %lf\n",
                        it.second->p1->id,
                        it.second->p2->id,
                        it.second->length,
                        it.second->minWidth);
                for (auto& point : it.second->maxPath) {
                    fprintf(fp, "p%d %d\n", point.x, point.y);
                }
                fprintf(fp, "e\n");
            }
            fclose(fp);
        }
    }
    {
        auto fp = fopen(path_nodes.c_str(), "w");
        if (fp) {
            for (auto& it : mesh.nodes) {
                fprintf(fp, "%d %d\n", it->position.x, it->position.y);
            }
            fclose(fp);
        }
    }
}
inline navmesh::navmesh* load(const std::string& path) {
    auto path_config = path + "/config.txt";
    auto path_ways = path + "/ways.txt";
    auto path_nodes = path + "/nodes.txt";
    auto path_vsdfMap = path + "/vsdfMap.chunk";
    auto path_sdfMap = path + "/sdfMap.chunk";
    auto path_pathDisMap = path + "/pathDisMap.chunk";
    auto path_pathNavMap = path + "/pathNavMap.chunk";
    auto path_idMap = path + "/idMap.chunk";

    int width, height;
    double minItemSize;
    auto fp_conf = fopen(path_config.c_str(), "r");
    bool haveFile = false;
    if (fp_conf) {
        if (fscanf(fp_conf, "%d %d %lf", &width, &height, &minItemSize) == 3) {
            haveFile = true;
        }
        fclose(fp_conf);
    }
    if (!haveFile) {
        return nullptr;
    }
    auto mesh = new navmesh::navmesh(width, height);
    mesh->minItemSize = minItemSize;

    loadMap(mesh->vsdfMap, path_vsdfMap);
    loadMap(mesh->sdfMap, path_sdfMap);
    loadMap(mesh->pathDisMap, path_pathDisMap);
    loadMap(mesh->pathNavMap, path_pathNavMap);
    loadMap(mesh->idMap, path_idMap);
    {
        auto fp = fopen(path_nodes.c_str(), "r");
        if (fp) {
            int index = 1;
            char buf[512];
            while (!feof(fp)) {
                int32_t x, y;
                bzero(buf, sizeof(buf));
                fgets(buf, sizeof(buf), fp);
                if (sscanf(buf, "%d %d", &x, &y) == 2) {
                    std::unique_ptr<navmesh::node> n(new navmesh::node);
                    n->id = index;
                    n->position.init(x, y);
                    mesh->nodes.push_back(std::move(n));
                    ++index;
                }
            }
            fclose(fp);
        }
    }
    {
        auto fp = fopen(path_ways.c_str(), "r");
        if (fp) {
            char buf[512];
            std::vector<ivec2> points;
            navmesh::node *p1 = nullptr, *p2 = nullptr;
            int id_p1 = 0;
            int id_p2 = 0;
            double length = 0;
            double minWidth = 0;
            while (!feof(fp)) {
                bzero(buf, sizeof(buf));
                fgets(buf, sizeof(buf), fp);
                switch (buf[0]) {
                    case 'e': {
                        if (p1 && p2) {
                            auto way_key = std::make_pair(id_p1, id_p2);
                            std::unique_ptr<navmesh::way> l(new navmesh::way);
                            l->p1 = p1;
                            l->p2 = p2;

                            l->length = length;

                            l->minWidth = minWidth;
                            l->maxPath = points;

                            p1->ways.insert(l.get());
                            p2->ways.insert(l.get());

                            mesh->ways[way_key] = std::move(l);
                            p1 = nullptr;
                            id_p1 = 0;
                            p2 = nullptr;
                            id_p2 = 0;
                            points.clear();
                        }
                    } break;
                    case 'c':
                        if (sscanf(&buf[1], "%d %d %lf %lf",
                                   &id_p1,
                                   &id_p2,
                                   &length,
                                   &minWidth) == 4) {
                            try {
                                p1 = mesh->nodes.at(id_p1 - 1).get();
                            } catch (...) {
                                printf("node no found!id=%d\n", id_p1);
                            }
                            try {
                                p2 = mesh->nodes.at(id_p2 - 1).get();
                            } catch (...) {
                                printf("node no found!id=%d\n", id_p2);
                            }
                        }
                        break;
                    case 'p': {
                        ivec2 tmp;
                        if (sscanf(&buf[1], "%d %d",
                                   &tmp.x,
                                   &tmp.y) == 2) {
                            points.push_back(tmp);
                        }
                    } break;
                }
            }
            fclose(fp);
        }
    }
    return mesh;
}

inline void savePoints(const std::vector<point_t>& points, const std::string& path) {
    auto fp = fopen(path.c_str(), "w");
    if (fp) {
        for (auto& it : points) {
            try {
                fprintf(fp, "%lf %lf\n",
                        it.at(0), it.at(1));
            } catch (...) {
            }
        }
        fclose(fp);
    }
}

inline void loadPoints(std::vector<point_t>& points, const std::string& path) {
    points.clear();
    auto fp = fopen(path.c_str(), "r");
    if (fp) {
        char buf[512];
        while (!feof(fp)) {
            double x, y;
            bzero(buf, sizeof(buf));
            fgets(buf, sizeof(buf), fp);
            if (sscanf(buf, "%lf %lf", &x, &y) == 2) {
                points.push_back({x, y});
            }
        }
        fclose(fp);
    }
}

}  // namespace sdpf::loader