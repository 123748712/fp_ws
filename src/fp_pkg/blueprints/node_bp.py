from flask import Blueprint, jsonify, request
from database.node_service import NodeService

node_bp = Blueprint('node_bp', __name__)
node_service = NodeService()

@node_bp.route('/getNodeType.do')
def get_nodeType():
    nodeType = node_service.get_nodeType()
    return jsonify(nodeType)

@node_bp.route('/getNodes.do', methods=['POST'])
def get_nodes():
    data = request.get_json()
    nodes = node_service.get_nodes(data)
    return jsonify(nodes)

@node_bp.route('/saveNode.do', methods=['POST'])
def save_node():
    node_data = request.get_json()
    try:
        result = node_service.save_node(node_data)

        if result:
            return jsonify({"result": "success", "message": "저장되었습니다."})
        else:
            return jsonify({"result": "fail", "message": "저장 중 오류가 발생했습니다."}), 500
    except Exception as e:
        print(f"Error: {e}")
        return jsonify({"result": "fail", "message": str(e)}), 500

@node_bp.route('/deleteNode.do', methods=['POST'])
def delete_node():
    node_data = request.get_json()
    try:
        result = node_service.delete_node(node_data)

        if result:
            return jsonify({"result": "success", "message": "삭제되었습니다."})
        else:
            return jsonify({"result": "fail", "message": "삭제 중 오류가 발생했습니다."}), 500
    except Exception as e:
        print(f"Error: {e}")
        return jsonify({"result": "fail", "message": str(e)}), 500
