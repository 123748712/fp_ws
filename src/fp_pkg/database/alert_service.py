from database.db_helper import DBHelper

class AlertService:
    def __init__(self):
        self.db = DBHelper()

    def get_alerts(self):
        sql = """
            select alert_type, alert_msg, alert_seq, check_yn,
            DATE_FORMAT(created_at, '%Y-%m-%d %H:%i:%s') as created_at
            from fp.tb_alert_info
            where del_yn = 'N'
            and check_yn = 'N'
            order by alert_seq desc
        """
        return self.db.fetch_all(sql)

    def insert_alert(self, alert):
        sql = """
            insert into fp.tb_alert_info (alert_type, alert_msg)
            values (%s, %s)
        """

        params = (
            alert['alert_type'],
            alert['alert_msg']
        )

        return self.db.execute_query(sql, params)

    def get_alert(self):
        sql = """
            select alert_type, alert_msg, alert_seq, check_yn, created_at
            from fp.tb_alert_info
            where del_yn = 'N'
            order by alert_seq desc
        """
        return self.db.fetch_all(sql)

    def check_alert(self, alert):
        sql = """
            update fp.tb_alert_info
            set check_yn = 'Y'
            where alert_seq = %s
        """

        params = (
            alert['alert_seq'],
        )

        return self.db.execute_query(sql, params)