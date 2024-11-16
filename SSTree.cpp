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

    float minDistance = std::numeric_limits<float>::max();
    SSNode* closestChild = nullptr;

    float distance;

    for (const auto& child : children)
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
    // TODO: Implementar actualización del sobre de delimitación.
    throw std::runtime_error("Not implemented yet");
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

    for (size_t i = 0; i < Point::getDimensions(); i++)
    {
        std::vector<float> values;
        for (const auto& centroid : centroids) { values.push_back(centroid[i]); }

        float mean = std::accumulate(values.begin(), values.end(), 0.0f) / values.size();
        float variance = 0.0f;
        for (const auto& value : values) { variance += (value - mean) * (value - mean); }
        variance /= values.size();

        if (variance > maxVariance)
        {
            maxVariance = variance;
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
SSNode* SSNode::split() {
    // TODO: Implementar división del nodo.
    throw std::runtime_error("Not implemented yet");
}

/**
 * findSplitIndex
 * Encuentra el índice de división en una coordenada específica.
 * @param coordinateIndex: Índice de la coordenada para encontrar el índice de división.
 * @return size_t: Índice de la división.
 */
size_t SSNode::findSplitIndex(size_t coordinateIndex) {
    // TODO: Implementar búsqueda del índice de división.
    throw std::runtime_error("Not implemented yet");
}

/**
 * getEntriesCentroids
 * Devuelve los centroides de las entradas.
 * Estos centroides pueden ser puntos almacenados en las hojas o los centroides de los nodos hijos en los nodos internos.
 * @return std::vector<Point>: Vector de centroides de las entradas.
 */
std::vector<Point> SSNode::getEntriesCentroids() {
    // TODO: Implementar obtención de centroides de las entradas.
    throw std::runtime_error("Not implemented yet");
}


/**
 * minVarianceSplit
 * Encuentra el índice de división óptimo para una lista de valores, de tal manera que la suma de las varianzas de las dos particiones resultantes sea mínima.
 * @param values: Vector de valores para encontrar el índice de mínima varianza.
 * @return size_t: Índice de mínima varianza.
 */
size_t SSNode::minVarianceSplit(const std::vector<float>& values) {
    // TODO: Implementar búsqueda del índice de mínima varianza.
    throw std::runtime_error("Not implemented yet");
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
SSNode* SSNode::insert(SSNode* node, Data* _data) {
    if (node->isLeaf)
    {
        for (const auto& data : node->_data)
        {
            if (data == _data) { return nullptr; }
        }

        node->_data.push_back(_data);
        node->updateBoundingEnvelope();

        if (node->_data.size() < node->maxPointsPerNode) { return nullptr; }
    }
    else
    {
        SSNode* closestChild = node->findClosestChild(_data->getEmbedding());
        SSNode* newNode = node->insert(closestChild, _data);

        if(newNode == nullptr) 
        {
            node->updateBoundingEnvelope();
            return nullptr;
        }

        node->children.push_back(newNode);
        node->updateBoundingEnvelope();

        if (node->children.size() <= node->maxPointsPerNode) { return nullptr; }
    }

    return node->split();
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
    // TODO: Implementar obtención del nodo raíz.
    throw std::runtime_error("Not implemented yet");
}

/**
 * insert
 * Inserta un dato en el árbol.
 * @param _data: Dato a insertar.
 */
void SSTree::insert(Data* _data) {
    // TODO: Implementar inserción en el árbol.
    throw std::runtime_error("Not implemented yet");
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
