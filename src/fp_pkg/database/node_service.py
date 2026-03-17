from database.db_helper import DBHelper

class NodeService:
    def __init__(self):
        self.db = DBHelper()

    def get_nodeType(self):
        sql = """
            select group_code, code_id, code_name, node_icon from fp.tb_common_code
            where group_code = 'NODE'
            and use_yn = 'y'
            order by sort_order asc
        """

        return self.db.fetch_all(sql)

    def get_nodes(self, data):
        sql = """
            select node_id, node_x_coord, node_y_coord from fp.tb_node_info
            where use_yn = 'Y'
            and map_id = %s
            order by node_id asc
        """
        params = {
            data['map_id'],
        }
        return self.db.fetch_all(sql, params)

    def save_node(self, node):
        ## 기존 데이터가 있으면 삭제하고 삭제가 실패하면 저장도 안되게
        delSuccess = self.delete_node(node)
        if not delSuccess:
            return False

        sql = """
            insert into tb_node_info (node_id, node_x_coord, node_y_coord, use_yn, map_id)
            values (%s, %s, %s, 'Y', %s)
        """
        params = (
            node['node_id'],
            node['node_x_coord'],
            node['node_y_coord'],
            node['map_id']
        )
        return self.db.execute_query(sql, params)

    def delete_node(self, node):
        sql = """
            update fp.tb_node_info
            set use_yn = 'N'
            where 1=1
            and node_id = %s
        """
        params = (
            node['node_id']
        )
        return self.db.execute_query(sql, params)