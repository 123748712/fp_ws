from flask import Blueprint, request, jsonify
from database.alert_service import AlertService

alert_bp = Blueprint('alert_bp', __name__)
alert_service = AlertService()

@alert_bp.route('/insertAlert.do', methods=['POST'])
def insert_alert():
    try:
        data = request.get_json()
        result = alert_service.insert_alert(data)
        if result:
            alerts = alert_service.get_alerts()
            return jsonify({"result": "success", "alerts": alerts})
        else:
            return jsonify({"result": "fail", "message": "저장 중 오류가 발생했습니다."}), 500
    except Exception as e:
        return jsonify({"result": "error", "message": str(e)})

@alert_bp.route('/getAlerts.do', methods=['GET'])
def get_alerts():
    alerts = alert_service.get_alerts()
    return jsonify(alerts)

@alert_bp.route('/checkAlert.do', methods=['POST'])
def check_alert():
    try:
        data = request.get_json()
        result = alert_service.check_alert(data)
        if result:
            alerts = alert_service.get_alerts()
            return jsonify({"result": "success", "alerts": alerts})
        else:
            return jsonify({"result": "fail", "message": "저장 중 오류가 발생했습니다."}), 500
    except Exception as e:
        return jsonify({"result": "error", "message": str(e)})