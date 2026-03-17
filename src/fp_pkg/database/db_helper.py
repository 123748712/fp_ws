import pymysql

class DBHelper:
    def __init__(self):
        self.DB_CONFIG = dict(
            host="192.168.0.57",
            user="dev",
            password="1234",
            database="fp",
            charset="utf8",
            cursorclass=pymysql.cursors.DictCursor
        )

    # 데이터 저장 공통 함수
    def execute_query(self, sql, params=None):
        conn = pymysql.connect(**self.DB_CONFIG)
        try:
            with conn.cursor() as cursor:
                cursor.execute(sql, params)
            conn.commit()
            return True
        except Exception as e:
            print(f"Query Error: {e}")
            conn.rollback()
            return False
        finally:
            conn.close()

    # 데이터 조회 공통 함수
    def fetch_all(self, sql, params=None):
        conn = pymysql.connect(**self.DB_CONFIG)
        try:
            with conn.cursor() as cursor:
                cursor.execute(sql, params)
                return cursor.fetchall()
        finally:
            conn.close()

    # 단일 데이터 조회 공통 함수
    def fetch_one(self, sql, params=None):
        conn = pymysql.connect(**self.DB_CONFIG)
        try:
            with conn.cursor() as cursor:
                cursor.execute(sql, params)
                return cursor.fetchone()
        finally:
            conn.close()

    # return seq
    def execute_query_seq(self, sql, params=None):
        conn = pymysql.connect(**self.DB_CONFIG)
        try:
            with conn.cursor() as cursor:
                cursor.execute(sql, params)
                last_id = cursor.lastrowid 
            conn.commit()
            return last_id
        except Exception as e:
            print(f"Query Error: {e}")
            conn.rollback()
            return None
        finally:
            conn.close()