#include "SSTree.h"

/**
 * intersectsPoint
 * Verifica si un punto está dentro de la esfera delimitadora del nodo.
 * @param point: Punto a verificar.
 * @return bool: Retorna true si el punto está dentro de la esfera, de lo contrario false.
 */
bool SSNode::intersectsPoint(const Point& point) const {
    return Point::distance(centroid, point) <= radius;
}

/**
 * findClosestChild
 * Encuentra el hijo más cercano a un punto dado.
 * @param target: El punto objetivo para encontrar el hijo más cercano.
 * @return SSNode*: Retorna un puntero al hijo más cercano.
 */
SSNode* SSNode::findClosestChild(const Point& target) {
    assert(!isLeaf && "findClosestChild should only be called on internal nodes.");

    float minDistance = std::numeric_limits<float>::infinity();
    SSNode* closestChild = nullptr;

    float distance;

    for (SSNode* child : children)
    {
        distance = Point::distance(child->getCentroid(), target);

        if (distance < minDistance)
        {
            minDistance = distance;
            closestChild = child;
        }
    }

    return closestChild;
}

/**
 * updateBoundingEnvelope
 * Actualiza el centroide y el radio del nodo basándose en los nodos internos o datos.
 */
void SSNode::updateBoundingEnvelope() {

    std::vector<Point> centroids = getEntriesCentroids();

    if (centroids.empty()) return;

    for (size_t i = 0; i < DIM; i++)
    {
        float sum = 0.0f;
        for (const auto& centroid : centroids) { sum += centroid[i]; }

        float mean = sum / centroids.size();
        centroid[i] = mean;
    }

    float maxEnvelope = 0.0f;

    if (this->isLeaf)
    {
        for (const Data* data : _data) {
            float distance = Point::distance(this->centroid, data->getEmbedding());
            maxEnvelope = std::max(maxEnvelope, distance);
        }
    }
    else
    {
        for (const SSNode* child : children)
        {
            float distance = Point::distance(this->centroid, child->getCentroid());
            float totalRadius = distance + child->getRadius();
            maxEnvelope = std::max(maxEnvelope, totalRadius);
        }
    }

    this->radius = maxEnvelope;
}

/**
 * varianceAlongDirection
 * Calcula la varianza de los centroides a lo largo de una dirección específica.
 * @param centroids: Vector de centroides.
 * @param directionIndex: Índice de la dirección para calcular la varianza.
 * @return float: Varianza a lo largo de la dirección.
 */
float varianceAlongDirection(const std::vector<Point>& centroids, size_t directionIndex) {
    float mean = 0.0f;
    float variance = 0.0f;

    for (const auto& centroid : centroids) { mean += centroid[directionIndex]; }
    mean = mean / centroids.size();

    for (const auto& centroid : centroids) { variance += (centroid[directionIndex] - mean) * (centroid[directionIndex] - mean); }
    variance = variance / centroids.size();

    return variance;
}

/**
 * directionOfMaxVariance
 * Calcula y retorna el índice de la dirección de máxima varianza.
 * @return size_t: Índice de la dirección de máxima varianza.
 */
size_t SSNode::directionOfMaxVariance() {
    float maxVariance = 0.0f;
    size_t directionIndex = 0;

    std::vector<Point> centroids = getEntriesCentroids();

    for (size_t i = 0; i < DIM; i++)
    {
        if (varianceAlongDirection(centroids, i) > maxVariance)
        {
            maxVariance = varianceAlongDirection(centroids, i);
            directionIndex = i;
        }
    }

    return directionIndex;
}

/**
 * split
 * Divide el nodo y retorna el nuevo nodo creado.
 * Implementación similar a R-tree.
 * @return SSNode*: Puntero al nuevo nodo creado por la división.
 */
std::pair<SSNode*, SSNode*> SSNode::split() {

    size_t splitIndex = this->findSplitIndex(this->directionOfMaxVariance());

    SSNode* newNode1 = new SSNode(this->centroid, this->maxPointsPerNode, this->radius, this->isLeaf, this->parent);
    SSNode* newNode2 = new SSNode(this->centroid, this->maxPointsPerNode, this->radius, this->isLeaf, this->parent);

    if (this->isLeaf)
    {
        newNode1->_data = std::vector<Data*>(this->_data.begin(), this->_data.begin() + splitIndex);
        newNode2->_data = std::vector<Data*>(this->_data.begin() + splitIndex, this->_data.end());
    }
    else
    {
        newNode1->children = std::vector<SSNode*>(this->children.begin(), this->children.begin() + splitIndex);
        newNode2->children = std::vector<SSNode*>(this->children.begin() + splitIndex, this->children.end());
    }

    newNode1->updateBoundingEnvelope();
    newNode2->updateBoundingEnvelope();

    return {newNode1, newNode2};
}

/**
 * findSplitIndex
 * Encuentra el índice de división en una coordenada específica.
 * @param coordinateIndex: Índice de la coordenada para encontrar el índice de división.
 * @return size_t: Índice de la división.
 */
size_t SSNode::findSplitIndex(size_t coordinateIndex) {
    
    std::vector<Point> centroids = getEntriesCentroids();

    std::sort(centroids.begin(), centroids.end(), [coordinateIndex](const Point& a, const Point& b) {
        return a[coordinateIndex] < b[coordinateIndex];
    });

    std::vector<float> points(centroids.size());
    std::transform(centroids.begin(), centroids.end(), points.begin(),
                   [coordinateIndex](const Point& point) { return point[coordinateIndex]; });

    return minVarianceSplit(points);
}


/**
 * getEntriesCentroids
 * Devuelve los centroides de las entradas.
 * Estos centroides pueden ser puntos almacenados en las hojas o los centroides de los nodos hijos en los nodos internos.
 * @return std::vector<Point>: Vector de centroides de las entradas.
 */
std::vector<Point> SSNode::getEntriesCentroids() {
    std::vector<Point> points;

    if (this->isLeaf)
    {
        for (const Data* data : this->_data) { points.push_back(data->getEmbedding()); }
    }
    else
    {
        for (const SSNode* child : this->children) { points.push_back(child->getCentroid()); }
    }

    return points;    
}

/**
 * variance
 * Calcula la varianza de un vector de valores.
 * @param values: Vector de valores para calcular la varianza.
 * @return float: Varianza de los valores.
 */
float variance(const std::vector<float>& values) {
    float mean = std::accumulate(values.begin(), values.end(), 0.0f) / values.size();
    float variance = 0.0f;

    for (const auto& value : values) { variance += (value - mean) * (value - mean); }
    variance = variance / values.size();

    return variance;
}

/**
 * minVarianceSplit
 * Encuentra el índice de división óptimo para una lista de valores, de tal manera que la suma de las varianzas de las dos particiones resultantes sea mínima.
 * @param values: Vector de valores para encontrar el índice de mínima varianza.
 * @return size_t: Índice de mínima varianza.
 */
size_t SSNode::minVarianceSplit(const std::vector<float>& values) {
    float minVariance = std::numeric_limits<float>::infinity();
    size_t splitIndex = 2;

    float variance1 = 0.0f;
    float variance2 = 0.0f;

    for (size_t i =  splitIndex; i < values.size() - splitIndex; i++)
    {
        variance1 = variance(std::vector<float>(values.begin(), values.begin() + i));
        variance2 = variance(std::vector<float>(values.begin() + i, values.end()));

        if (variance1 + variance2 < minVariance)
        {
            minVariance = variance1 + variance2;
            splitIndex = i;
        }
    }

    return splitIndex;
}

/**
 * searchParentLeaf
 * Busca el nodo hoja adecuado para insertar un punto.
 * @param node: Nodo desde el cual comenzar la búsqueda.
 * @param target: Punto objetivo para la búsqueda.
 * @return SSNode*: Nodo hoja adecuado para la inserción.
 */
SSNode* SSNode::searchParentLeaf(SSNode* node, const Point& target) {
    if (node->isLeaf) return node;

    SSNode* child = node->findClosestChild(target);
    return searchParentLeaf(child, target);
}

/**
 * insert
 * Inserta un dato en el nodo, dividiéndolo si es necesario.
 * @param node: Nodo donde se realizará la inserción.
 * @param _data: Dato a insertar.
 * @return SSNode*: Nuevo nodo raíz si se dividió, de lo contrario nullptr.
 */
std::pair<SSNode*, SSNode*> SSNode::insert(SSNode* node, Data* __data) {

    if (this->isLeaf)
    {
        for (const Data* data : this->_data)
        {
            if (data == __data) { return {nullptr, nullptr}; }
        }

        this->_data.push_back(__data);
        this->updateBoundingEnvelope();

        if (this->_data.size() <= this->maxPointsPerNode) { return {nullptr, nullptr}; }
    }
    else
    {
        SSNode* closestChild = this->findClosestChild(__data->getEmbedding());

        auto [newNode1, newNode2] = closestChild->insert(closestChild, __data);

        if (newNode1 == nullptr) 
        {
            node->updateBoundingEnvelope();
            return {nullptr, nullptr};
        }
        else
        {
            this->children.erase(std::remove(this->children.begin(), this->children.end(), closestChild), this->children.end());
            this->children.push_back(newNode1);
            this->children.push_back(newNode2);

            this->updateBoundingEnvelope();

            if (this->children.size() <= this->maxPointsPerNode) { return {nullptr, nullptr}; }
        }
    }

    return this->split();
}


/**
 * search
 * Busca un dato específico en el árbol.
 * @param node: Nodo desde el cual comenzar la búsqueda.
 * @param _data: Dato a buscar.
 * @return SSNode*: Nodo que contiene el dato (o nullptr si no se encuentra).
 */
SSNode* SSNode::search(SSNode* node, Data* _data) {
    if (node == nullptr) return nullptr;

    if (node->isLeaf)
    {
        for (const auto& data : node->_data)
        {
            if (data == _data) { return node; }
        }
    }
    else
    {
        SSNode* result = nullptr;
        for (const auto& child : node->children)
        {
            if (child->intersectsPoint(_data->getEmbedding()))
            {
                result = search(child, _data);
                if (result != nullptr) { return result; }
            }
        }
    }

    return nullptr;
}


/**
 * getRoot
 * Retorna el nodo raíz del árbol.
 * @return SSNode*: Nodo raíz del árbol.
 */
SSNode* SSTree::getRoot() const {
    return root;
}

/**
 * insert
 * Inserta un dato en el árbol.
 * @param _data: Dato a insertar.
 */
void SSTree::insert(Data* _data) {
    if (root == nullptr) {
        root = new SSNode(_data->getEmbedding(), maxPointsPerNode, 0.0f, true, nullptr);
    }

    auto [newNode1, newNode2] = root->insert(root, _data);
    if (newNode1 == nullptr) return;

    SSNode* newRoot = new SSNode(root->getCentroid(), maxPointsPerNode, root->getRadius(), false, nullptr);
    newRoot->children.push_back(newNode1);
    newRoot->children.push_back(newNode2);

    newRoot->updateBoundingEnvelope();
    root = newRoot;
}

/**
 * search
 * Busca un dato específico en el árbol.
 * @param _data: Dato a buscar.
 * @return SSNode*: Nodo que contiene el dato (o nullptr si no se encuentra).
 */
SSNode* SSTree::search(Data* _data) {
    return root->search(root, _data);
}

/**
 * depthFirstSearch
 * Realiza una búsqueda en profundidad en el árbol.
 * @param q: Punto de consulta.
 * @param k: Número de vecinos más cercanos a buscar.
 * @param e: Nodo actual.
 * @param pq: Cola de prioridad para almacenar los vecinos más cercanos.
 * @param Dk: Distancia al k-ésimo vecino más cercano.
 */
void SSNode::depthFirstSearch( const Point& q, const int& k, SSNode* e, std::priority_queue<Data*, std::vector<Data*>, PQCompare>& L, float& Dk) {
    
    if (e->isLeaf)
    {
        float queryToCentroid = Point::distance(e->getCentroid(), q);

        for (const Data* data : e->_data) {

            float dataToCentroid = Point::distance(data->getEmbedding(), e->getCentroid());

            if (queryToCentroid - dataToCentroid > Dk || 
                dataToCentroid - queryToCentroid > Dk) continue;

            float queryToData = Point::distance(data->getEmbedding(), q);
            if (queryToData < Dk) {
                L.push(const_cast<Data*>(data));
                if (static_cast<int>(L.size()) > k) L.pop();
                if (static_cast<int>(L.size()) == k) Dk = Point::distance(L.top()->getEmbedding(), q);
            }
        }
    } 
    else
    {
        std::sort(
            e->children.begin(), e->children.end(),
            [&q](SSNode* a, SSNode* b) {
                return Point::distance(a->getCentroid(), q) < Point::distance(b->getCentroid(), q);
            }
        );
        
        if (k == 1) {
            for (SSNode* child : e->children) {

                float distToCentroid = Point::distance(child->getCentroid(), q);
                if (distToCentroid > Dk) break;
                if (distToCentroid + child->getRadius() < Dk) {
                    Dk = distToCentroid + child->getRadius();
                    continue;
                }
                depthFirstSearch(q, k, child, L, Dk);
            }
        } else {
            for (SSNode* child : e->children) {
                float distToCentroid = Point::distance(child->getCentroid(), q);
                if (distToCentroid - child->getRadius() > Dk) continue;
                if (child->getRadius() - distToCentroid > Dk) continue;
                depthFirstSearch(q, k, child, L, Dk);
            }
        }
    }

    return;
}

/**
 * depthFirstSearch
 * Realiza una búsqueda en profundidad en el árbol.
 * @param q: Punto de consulta.
 * @return std::vector<Data*>: Vector de datos que se encuentran en la búsqueda.
 */
// Implementación de SSTree::depthFirstSearch
std::vector<Data*> SSTree::depthFirstSearch(const Point& q, const int& k) const {
    PQCompare comparator(q);
    std::priority_queue<Data*, std::vector<Data*>, PQCompare> L(comparator);
    float Dk = std::numeric_limits<float>::infinity();

    if (root) {
        root->depthFirstSearch(q, k, root, L, Dk);
    }

    std::vector<Data*> result;
    while (!L.empty()) {
        result.push_back(L.top());
        L.pop();
    }
    std::reverse(result.begin(), result.end());
    return result;
}
