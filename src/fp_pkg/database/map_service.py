from database.db_helper import DBHelper
from utils.map_manager import MapManager

class MapService:
    def __init__(self):
        self.db = DBHelper()
        self.map_mgr = MapManager()


    # 현재 선택한 맵 조회
    def get_map(self, map):
        sql = """
            select map_seq, map_name, map_file_path from fp.tb_map_info
            where map_seq = %s
            and use_yn = 'Y'
        """
        params = (
            map['map_seq'],
        )
        return self.db.fetch_one(sql, params)
    
    # 현재 active상태인 맵 조회
    def get_active_map(self):
        sql = """
            select map_seq, map_name, map_file_path from fp.tb_map_info
            where active_yn = 'Y'
            and use_yn = 'Y'
        """
       
        return self.db.fetch_one(sql)
    
    # 맵 리스트 조회
    def get_maps(self):
        sql = """
            select map_seq, map_name, map_file_path, active_yn from fp.tb_map_info
            where use_yn = 'Y'
            order by map_seq asc
        """
        return self.db.fetch_all(sql)

    def change_map_status(self, map):
        sql = """
            update fp.tb_map_info
            set active_yn =
                case when map_seq = %s then 'Y' else 'N'
            end
        """
        params = (
            map["map_seq"],
        )
        return self.db.execute_query(sql, params)

    def save_map(self, map):
        # 모든 맵 비활성화
        data = {"map_seq" : 0}
        self.change_map_status(data)

        sql = """
            insert into fp.tb_map_info (map_name, map_file_path, use_yn, active_yn)
            values (%s, %s, 'Y', 'Y')

        """
        params = (
            map["map_name"],
            map["map_file_path"]
        )
        
        seq = self.db.execute_query_seq(sql, params)

        if seq is None:
            return False

        return {"map_seq": seq, "map_name": map['map_name'], "map_file_path": map['map_file_path']}

    def delete_map(self, map):
        sql = """
            update fp.tb_map_info
            set active_yn = case when map_seq = 1 then 'Y' else 'N' end,
                use_yn = case when map_seq = %s then 'N' else use_yn end
        """
        params = (
            map["map_seq"],
        )
        return self.db.execute_query(sql, params)

map_service = MapService()