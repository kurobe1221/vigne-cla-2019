#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <time.h>
#include <limits.h>

//////////////////////////////
// マクロ・定数
//////////////////////////////
#define OBJECT_SIZE 4000
#define SCALE_SIZE 20
#define GENERATOR_FILE_NAME "generators.txt"
#define EQUIPMENT_FILE_NAME "equipments.txt"
#define EXPORT_FILE_PREFIX "route"
#define ZX_WIDTH_MIN 3
#define ZX_WIDTH_MAX 5
#define GENERATION 30000
#define INDEX(x,y,z) (((x) * SCALE_SIZE * SCALE_SIZE) + ((y) * SCALE_SIZE) + (z))
#define IS_OPEN(rt)  ((rt)->prev == -1 && (rt)->next == -1)
#define IS_CLOSE(rt) ((rt)->prev != -1 || (rt)->next != -1)
#define IS_HEAD(rt)  ((rt)->prev == -1 && (rt)->next != -1)
#define IS_TAIL(rt)  ((rt)->prev != -1 && (rt)->next == -1)

//////////////////////////////
// 型定義
//////////////////////////////

typedef struct {
  int x; // X座標
  int y; // Y座標
  int z; // Z座標
} xyz_t;

typedef struct {
  xyz_t gen; // 発電機の座標
  xyz_t equ; // 装置の座標
  int cost;  // 二点間のコスト
} object_t;

typedef struct {
  int prev;    // 前のINDEX
  int next;    // 次のINDEX
  xyz_t coord; // 座標
} route_t;

typedef struct {
  int key;   // KEY
  int value; // VALUE
} qsort_t;

//////////////////////////////
// プロトタイプ宣言
//////////////////////////////
object_t* create_object_list();
int dist(int const _gen_x, int const _gen_y, int const _gen_z,
          int const _equ_x, int const _equ_y, int const _equ_z);
int* create_object_no_list(object_t* const _object_list);
route_t* create_route_list();
int qsort_desc(void const* const _a, void const* const _b);
qsort_t* create_cost_list(object_t const* const _object_list);
void search_route(object_t const* const _object_list, int const* const _object_no_list,
                  qsort_t const* const _cost_list, route_t* const _route_list);
void search_route_by_object_no(object_t const* const _object_list,
                                int const* const _object_no_list,
                                int const _object_no, route_t* const _route_list);
void search_route_by_kopt(object_t const* const _object_list,  
                          int const* const _object_no_list,
                          int const _from_x, int const _from_y, int const _from_z,
                          int const _to_x, int const _to_y, int const _to_z,
                          route_t* const _route_list);
void kopt_local_search(object_t const* const _object_list,  
                        int const* const _object_no_list,
                        int const _from_x, int const _from_y, int const _from_z,
                        int const _to_x, int const _to_y, int const _to_z,
                        int const _depth, int const _total_cost,
                        int* const _best_cost, int* const _best_route,
                        route_t* const _route_list);
void search_route_by_object_no_multi_way(object_t const* const _object_list,
                                          int const* const _object_no_list,
                                          int const _object_no, route_t* const _route_list);
void search_route_by_kopt_multi_way(object_t const* const _object_list,  
                                    int const* const _object_no_list,
                                    int const _from_x, int const _from_y, int const _from_z,
                                    int const _to_x, int const _to_y, int const _to_z,
                                    route_t* const _route_list);
void kopt_multi_way_local_search(object_t const* const _object_list,  
                                  int const* const _object_no_list,
                                  int const _from_x, int const _from_y, int const _from_z,
                                  int const _to_x, int const _to_y, int const _to_z,
                                  int const _depth, int const _total_cost,
                                  int* const _best_cost, int* const _best_route,
                                  route_t* const _route_list);
void kopt_next_call(object_t const* const _object_list,  
                    int const* const _object_no_list,
                    int const _from_x, int const _from_y, int const _from_z,
                    int const _next_x, int const _next_y, int const _next_z,
                    int const _to_x, int const _to_y, int const _to_z,
                    int const _depth, int const _total_cost,
                    int* const _best_cost, int* const _best_route,
                    route_t* const _route_list);
void kopt_best_judge(int const _x, int const _y, int const _z,
                      int const _total_cost, int* const _best_cost,
                      int* const _best_route, route_t* const _route_list);
int count_route(object_t const* const _object_list, route_t const* const _route_list);
void plot(object_t const* const _object_list, route_t const* const _route_list);
void export_route(object_t const* const _object_list,
                  route_t const* const _route_list, char const* const _file_name);
int route_file_check(int const* const _object_no_list, char const* const _file_name);
void initrand(uint32_t seed);
double urand();
void zoning_remove(route_t const* const _route_parent, route_t* const _route_child);
void zx_crossover(object_t const* const _object_list, int const* const _object_no_list, qsort_t const* const _cost_list,
                  route_t const* const _route_parent_1, route_t const* const _route_parent_2,
                  route_t* const _route_child_1, route_t* const _route_child_2);
int zx_inner_zone(int const _begin_x, int const _end_x,
                  int const _begin_y, int const _end_y,
                  int const _begin_z, int const _end_z,
                  int const _route_index, route_t const* const _route_list);
void zx_remove_route(int const _route_index, route_t* const _route_list);

//////////////////////////////
// エントリーポイント
//////////////////////////////
int main() {
  char export_file_name[256];
  int i, j, r, total_parent, total_child;
  object_t* object_list = NULL;
  int* object_no_list = NULL;
  route_t* route_parent = NULL;
  route_t* route_child = NULL;
  route_t* route_temp = NULL;
  qsort_t* cost_list = NULL;
  qsort_t tmp;

  // ランダム関数の初期化
  initrand((unsigned int)time(NULL));

  // オブジェクト（発電機・装置）リスト生成
  printf("create object list ... ");
  if((object_list = create_object_list()) == NULL) {
    return -1;
  }
  printf("ok\n");

  // オブジェクトNo.リスト生成
  printf("create object No. list ... ");
  if((object_no_list = create_object_no_list(object_list)) == NULL) {
    return -1;
  }
  printf("ok\n");

  // コストリスト生成
  printf("create cost list ... ");
  if((cost_list = create_cost_list(object_list)) == NULL) {
    return -1;
  }
  printf("ok\n");

  // 順路リスト生成
  printf("create route list ... ");
  if((route_parent = create_route_list()) == NULL) {
    return -1;
  }
  if((route_child = create_route_list()) == NULL) {
    return -1;
  }
  printf("ok\n");

  // 順路検索
  printf("search route ... ");
  search_route(object_list, object_no_list, cost_list, route_parent);
  printf("ok\n");

  // 順路数の表示
  total_parent = count_route(object_list, route_parent);
  printf("total = %d\n", total_parent);

  // 順路探索ループ
  for(i = 0; i < GENERATION; ++ i) {
    // ゾーン削除
    zoning_remove(route_parent, route_child);
    // ランダムにシャッフル
    for(j = 0; j < OBJECT_SIZE; ++ j) {
      r = (int)(urand() * (double)OBJECT_SIZE);
      tmp = cost_list[j];
      cost_list[j] = cost_list[r];
      cost_list[r] = tmp;
    }
    // コストが高い順にソート
    qsort(cost_list, OBJECT_SIZE, sizeof(qsort_t), qsort_desc);
    // 順路再構築
    search_route(object_list, object_no_list, cost_list, route_child);
    // 順路数の表示
    total_child = count_route(object_list, route_child);
    // 更新
    if(total_child > total_parent) {
      printf("update total = %d, i = %d\n", total_child, i);
      i = -1;
      sprintf(export_file_name, "%s_%d.txt", EXPORT_FILE_PREFIX, total_child);
      export_route(object_list, route_child, export_file_name);

      total_parent = total_child;
      route_temp = route_parent;
      route_parent = route_child;
      route_child = route_temp;
    }
  }
  
  // 順路数の表示
  total_parent = count_route(object_list, route_parent);
  printf("total = %d\n", total_parent);

  // グラフ表示
  printf("plot ... ");
  plot(object_list, route_parent);
  printf("ok\n");

  // 順路をエクスポート
  printf("export route ... ");
  sprintf(export_file_name, "%s_%d.txt", EXPORT_FILE_PREFIX, total_parent);
  export_route(object_list, route_parent, export_file_name);
  printf("ok\n");

  // 順路ファイルチェック
  printf("route check %s ... ", export_file_name);
  if(route_file_check(object_no_list, export_file_name) == 0) {
    printf("ok\n");
  }

  // メモリ開放
  free(object_list);
  free(object_no_list);
  free(route_parent);
  free(route_child);
  free(cost_list);
  return 0;
}

//////////////////////////////
// オブジェクト（発電機・装置）リスト生成
//////////////////////////////
object_t* create_object_list() {
  object_t* mem = NULL;
  FILE* fp_gen;
  FILE* fp_equ;
  int i;
  object_t* obj;

  // メモリ確保
  if((mem = (object_t*)malloc(sizeof(object_t) * OBJECT_SIZE)) == NULL) {
    return NULL;
  }
  // 発電機の座標ファイル
  if((fp_gen = fopen(GENERATOR_FILE_NAME, "r")) == NULL) {
    free(mem);
    return NULL;
  }
  // 装置の座標ファイル
  if((fp_equ = fopen(EQUIPMENT_FILE_NAME, "r")) == NULL) {
    free(mem);
    fclose(fp_gen);
    return NULL;
  }
  // 座標読み込み
  for(i = 0; i < OBJECT_SIZE; ++ i) {
    obj = mem + i;
    fscanf(fp_gen, "%d %d %d", &(obj->gen.x), &(obj->gen.y), &(obj->gen.z));
    fscanf(fp_equ, "%d %d %d", &(obj->equ.x), &(obj->equ.y), &(obj->equ.z));
    // コスト
    obj->cost = (SCALE_SIZE + SCALE_SIZE + SCALE_SIZE)
              - dist(obj->gen.x, obj->gen.y, obj->gen.z, obj->equ.x, obj->equ.y, obj->equ.z);
    // DEBUG
    // printf("obj[%04d] gen: %02d %02d %02d | equ: %02d %02d %02d | cost: %d\n",
    //        i, obj->gen.x, obj->gen.y, obj->gen.z,
    //        obj->equ.x, obj->equ.y, obj->equ.z, obj->cost);
  } 
  fclose(fp_gen);
  fclose(fp_equ);
  return mem;
}

//////////////////////////////
// 二点間の最短距離を求める
//////////////////////////////
int dist(int const _gen_x, int const _gen_y, int const _gen_z,
          int const _equ_x, int const _equ_y, int const _equ_z) {
  return abs(_gen_x - _equ_x) + abs(_gen_y - _equ_y) + abs(_gen_z - _equ_z);
}

//////////////////////////////
// オブジェクトNo.リスト生成
//////////////////////////////
int* create_object_no_list(object_t* const _object_list) {
  int* mem = NULL;
  int i;
  object_t* obj;
  // DEBUG
  // int x, y, z;

  // メモリ確保
  if((mem = (int*)malloc(sizeof(int) * SCALE_SIZE * SCALE_SIZE * SCALE_SIZE)) == NULL) {
    return NULL;
  }
  // No.紐付け
  for(i = 0; i < OBJECT_SIZE; ++ i) {
    obj = _object_list + i;
    mem[INDEX(obj->gen.x, obj->gen.y, obj->gen.z)] = i;
    mem[INDEX(obj->equ.x, obj->equ.y, obj->equ.z)] = i;
  }
  // DEBUG
  // for(x = 0; x < SCALE_SIZE; ++ x) {
  //   for(y = 0; y < SCALE_SIZE; ++ y) {
  //     for(z = 0; z < SCALE_SIZE; ++ z) {
  //       printf("[%02d][%02d][%02d] = %d\n", x, y, z, mem[INDEX(x, y, z)]);
  //     }
  //   }
  // }
  return mem;
}

//////////////////////////////
// ソート用比較関数
//////////////////////////////
int qsort_desc(void const* const _a, void const* const _b) {
  qsort_t const* const a = (qsort_t const*)_a;
  qsort_t const* const b = (qsort_t const*)_b;
  return b->value - a->value;
}

//////////////////////////////
// コストリスト生成
//////////////////////////////
qsort_t* create_cost_list(object_t const* const _object_list) {
  int i, r;
  qsort_t* mem = NULL;
  qsort_t* qs;
  qsort_t tmp;

  // ソート用配列生成
  if((mem = (qsort_t*)malloc(sizeof(qsort_t) * OBJECT_SIZE)) == NULL) {
    return NULL;
  }
  // key と value をセット
  for(i = 0; i < OBJECT_SIZE; ++ i) {
    qs = mem + i;
    qs->key = i;
    qs->value = _object_list[i].cost;
  }
  // ランダムにシャッフル
  for(i = 0; i < OBJECT_SIZE; ++ i) {
    r = (int)(urand() * (double)OBJECT_SIZE);
    tmp = mem[i];
    mem[i] = mem[r];
    mem[r] = tmp;
  }
  // コストが高い順にソート
  qsort(mem, OBJECT_SIZE, sizeof(qsort_t), qsort_desc);
  return mem;
}

//////////////////////////////
// 順路リスト生成
//////////////////////////////
route_t* create_route_list() {
  int x, y, z;
  route_t* mem = NULL;
  route_t* rt;

  // メモリ確保
  if((mem = (route_t*)malloc(sizeof(route_t) * SCALE_SIZE * SCALE_SIZE * SCALE_SIZE)) == NULL) {
    return NULL;
  }
  // 初期化
  for(x = 0; x < SCALE_SIZE; ++ x) {
    for(y = 0; y < SCALE_SIZE; ++ y) {
      for(z = 0; z < SCALE_SIZE; ++ z) {
        rt = mem + INDEX(x, y, z);
        rt->prev = -1;
        rt->next = -1;
        rt->coord.x = x;
        rt->coord.y = y;
        rt->coord.z = z;
      }
    }
  }
  return mem;
}

//////////////////////////////
// 順路探索
//////////////////////////////
void search_route(object_t const* const _object_list, int const* const _object_no_list,
                  qsort_t const* const _cost_list, route_t* const _route_list) {
  int i;
  object_t const* obj;
  qsort_t const* cost;
  route_t* rt_gen;
  route_t* rt_equ;

  // 順番に探索（局所）
  for(i = 0; i < OBJECT_SIZE; ++ i) {
    cost = _cost_list + i;
    obj = _object_list + cost->key;
    rt_gen = _route_list + INDEX(obj->gen.x, obj->gen.y, obj->gen.z);
    rt_equ = _route_list + INDEX(obj->equ.x, obj->equ.y, obj->equ.z);
    // 単方向探索
    if(IS_OPEN(rt_gen) && IS_OPEN(rt_equ)) {
      search_route_by_object_no(_object_list, _object_no_list, cost->key, _route_list);      
    }
    // 全方向探索
    if(IS_OPEN(rt_gen) && IS_OPEN(rt_equ)) {
      search_route_by_object_no_multi_way(_object_list, _object_no_list, cost->key, _route_list);      
    }
  }
}

//////////////////////////////
// オブジェクトNo.を指定して順路探索
//////////////////////////////
void search_route_by_object_no(object_t const* const _object_list,
                                int const* const _object_no_list,
                                int const _object_no, route_t* const _route_list) {
  object_t const* obj;

  // 発電機と装置をつなぐ順路探索
  obj = _object_list + _object_no;
  search_route_by_kopt(_object_list, _object_no_list,
                        obj->gen.x, obj->gen.y, obj->gen.z,
                        obj->equ.x, obj->equ.y, obj->equ.z,
                        _route_list);
}

//////////////////////////////
// K-OPT 順路探索
//////////////////////////////
void search_route_by_kopt(object_t const* const _object_list,  
                          int const* const _object_no_list,
                          int const _from_x, int const _from_y, int const _from_z,
                          int const _to_x, int const _to_y, int const _to_z,
                          route_t* const _route_list) {
  int best_route[SCALE_SIZE + SCALE_SIZE + SCALE_SIZE];
  int best_cost;
  int* index_curr;
  int* index_prev;
  route_t* rt_curr;
  route_t* rt_prev;
  
  // 順路探索
  best_cost = INT_MAX;
  kopt_local_search(_object_list, _object_no_list,
                    _from_x, _from_y, _from_z,
                    _to_x, _to_y, _to_z,
                    1, 0, &best_cost,
                    best_route, _route_list);
  // 順路が見つかった場合
  if(best_cost < INT_MAX) {
    // 順路を逆から構築
    index_curr = best_route;
    index_prev = best_route + 1;
    while(*index_prev != -1) {
      rt_curr = _route_list + *index_curr;
      rt_prev = _route_list + *index_prev;
      rt_curr->prev = *index_prev;
      rt_prev->next = *index_curr;
      ++ index_curr;
      ++ index_prev;
    }
  }
}

//////////////////////////////
// K-OPT 局所的な順路探索
//////////////////////////////
void kopt_local_search(object_t const* const _object_list,  
                        int const* const _object_no_list,
                        int const _from_x, int const _from_y, int const _from_z,
                        int const _to_x, int const _to_y, int const _to_z,
                        int const _depth, int const _total_cost,
                        int* const _best_cost, int* const _best_route,
                        route_t* const _route_list) {
  int delta_x, delta_y, delta_z;
 
  // 順路が目的地点に到達した場合、記録判定と更新
  if(_from_x == _to_x && _from_y == _to_y && _from_z == _to_z) {
    kopt_best_judge(_from_x, _from_y, _from_z, _total_cost,
                    _best_cost, _best_route, _route_list);
    return;
  }
  // Z移動（プラス方向）
  delta_z = _to_z - _from_z;
  if(delta_z > 0) {
    kopt_next_call(_object_list, _object_no_list,
                    _from_x, _from_y, _from_z,
                    _from_x, _from_y, _from_z + 1,
                    _to_x, _to_y, _to_z,
                    _depth, _total_cost, _best_cost,
                    _best_route, _route_list);
  }
  // Z移動（マイナス方向）
  else if(delta_z < 0) {
    kopt_next_call(_object_list, _object_no_list,
                    _from_x, _from_y, _from_z,
                    _from_x, _from_y, _from_z - 1,
                    _to_x, _to_y, _to_z,
                    _depth, _total_cost, _best_cost,
                    _best_route, _route_list);
  }

  // Y移動（プラス方向）
  delta_y = _to_y - _from_y;
  if(delta_y > 0) {
    kopt_next_call(_object_list, _object_no_list,
                    _from_x, _from_y, _from_z,
                    _from_x, _from_y + 1, _from_z,
                    _to_x, _to_y, _to_z,
                    _depth, _total_cost, _best_cost,
                    _best_route, _route_list);
  }
  // Y移動（マイナス方向）
  else if(delta_y < 0) {
    kopt_next_call(_object_list, _object_no_list,
                    _from_x, _from_y, _from_z,
                    _from_x, _from_y - 1, _from_z,
                    _to_x, _to_y, _to_z,
                    _depth, _total_cost, _best_cost,
                    _best_route, _route_list);
  }

  // X移動（プラス方向）
  delta_x = _to_x - _from_x;
  if(delta_x > 0) {
    kopt_next_call(_object_list, _object_no_list,
                    _from_x, _from_y, _from_z,
                    _from_x + 1, _from_y, _from_z,
                    _to_x, _to_y, _to_z,
                    _depth, _total_cost, _best_cost,
                    _best_route, _route_list);
  }
  // X移動（マイナス方向）
  else if(delta_x < 0) {
    kopt_next_call(_object_list, _object_no_list,
                    _from_x, _from_y, _from_z,
                    _from_x - 1, _from_y, _from_z,
                    _to_x, _to_y, _to_z,
                    _depth, _total_cost, _best_cost,
                    _best_route, _route_list);
  }
}

//////////////////////////////
// オブジェクトNo.を指定して順路探索（全方向）
//////////////////////////////
void search_route_by_object_no_multi_way(object_t const* const _object_list,
                                          int const* const _object_no_list,
                                          int const _object_no, route_t* const _route_list) {
  object_t const* obj;

  // 発電機と装置をつなぐ順路探索
  obj = _object_list + _object_no;
  search_route_by_kopt_multi_way(_object_list, _object_no_list,
                                  obj->gen.x, obj->gen.y, obj->gen.z,
                                  obj->equ.x, obj->equ.y, obj->equ.z,
                                  _route_list);
}

//////////////////////////////
// K-OPT 順路探索
//////////////////////////////
void search_route_by_kopt_multi_way(object_t const* const _object_list,  
                                    int const* const _object_no_list,
                                    int const _from_x, int const _from_y, int const _from_z,
                                    int const _to_x, int const _to_y, int const _to_z,
                                    route_t* const _route_list) {
  int best_route[SCALE_SIZE + SCALE_SIZE + SCALE_SIZE];
  int best_cost;
  int* index_curr;
  int* index_prev;
  route_t* rt_curr;
  route_t* rt_prev;

  // 順路探索
  best_cost = INT_MAX;
  kopt_multi_way_local_search(_object_list, _object_no_list,
                              _from_x, _from_y, _from_z,
                              _to_x, _to_y, _to_z,
                              1, 0, &best_cost,
                              best_route, _route_list);
  // 順路が見つかった場合
  if(best_cost < INT_MAX) {
    // 順路を逆から構築
    index_curr = best_route;
    index_prev = best_route + 1;
    while(*index_prev != -1) {
      rt_curr = _route_list + *index_curr;
      rt_prev = _route_list + *index_prev;
      rt_curr->prev = *index_prev;
      rt_prev->next = *index_curr;
      ++ index_curr;
      ++ index_prev;
    }
  }
}

//////////////////////////////
// K-OPT 局所的な順路探索（全方向）
//////////////////////////////
void kopt_multi_way_local_search(object_t const* const _object_list,  
                                  int const* const _object_no_list,
                                  int const _from_x, int const _from_y, int const _from_z,
                                  int const _to_x, int const _to_y, int const _to_z,
                                  int const _depth, int const _total_cost,
                                  int* const _best_cost, int* const _best_route,
                                  route_t* const _route_list) {
  // 順路が目的地点に到達した場合、記録更新
  if(_from_x == _to_x && _from_y == _to_y && _from_z == _to_z) {
    kopt_best_judge(_from_x, _from_y, _from_z, _total_cost,
                    _best_cost, _best_route, _route_list);
    return;
  }
  // Z移動（プラス方向）
  if(_from_z + 1 < SCALE_SIZE) {
    kopt_next_call(_object_list, _object_no_list,
                    _from_x, _from_y, _from_z,
                    _from_x, _from_y, _from_z + 1,
                    _to_x, _to_y, _to_z,
                    _depth, _total_cost, _best_cost,
                    _best_route, _route_list);
  }
  // Z移動（マイナス方向）
  if(_from_z - 1 >= 0) {
    kopt_next_call(_object_list, _object_no_list,
                    _from_x, _from_y, _from_z,
                    _from_x, _from_y, _from_z - 1,
                    _to_x, _to_y, _to_z,
                    _depth, _total_cost, _best_cost,
                    _best_route, _route_list);
  }

  // Y移動（プラス方向）
  if(_from_y + 1 < SCALE_SIZE) {
    kopt_next_call(_object_list, _object_no_list,
                    _from_x, _from_y, _from_z,
                    _from_x, _from_y + 1, _from_z,
                    _to_x, _to_y, _to_z,
                    _depth, _total_cost, _best_cost,
                    _best_route, _route_list);
  }
  // Y移動（マイナス方向）
  if(_from_y - 1 >= 0) {
    kopt_next_call(_object_list, _object_no_list,
                    _from_x, _from_y, _from_z,
                    _from_x, _from_y - 1, _from_z,
                    _to_x, _to_y, _to_z,
                    _depth, _total_cost, _best_cost,
                    _best_route, _route_list);
  }

  // X移動（プラス方向）
  if(_from_x + 1 < SCALE_SIZE) {
    kopt_next_call(_object_list, _object_no_list,
                    _from_x, _from_y, _from_z,
                    _from_x + 1, _from_y, _from_z,
                    _to_x, _to_y, _to_z,
                    _depth, _total_cost, _best_cost,
                    _best_route, _route_list);
  }
  // X移動（マイナス方向）
  if(_from_x - 1 >= 0) {
    kopt_next_call(_object_list, _object_no_list,
                    _from_x, _from_y, _from_z,
                    _from_x - 1, _from_y, _from_z,
                    _to_x, _to_y, _to_z,
                    _depth, _total_cost, _best_cost,
                    _best_route, _route_list);
  }
}

//////////////////////////////
// K-OPT 次の順路探索
//////////////////////////////
void kopt_next_call(object_t const* const _object_list,  
                    int const* const _object_no_list,
                    int const _from_x, int const _from_y, int const _from_z,
                    int const _next_x, int const _next_y, int const _next_z,
                    int const _to_x, int const _to_y, int const _to_z,
                    int const _depth, int const _total_cost,
                    int* const _best_cost, int* const _best_route,
                    route_t* const _route_list) {
  int index_from, index_next, object_no, cost;
  route_t* rt_from;
  route_t* rt_next;
  route_t* rt_gen;
  route_t* rt_equ;
  object_t const* obj;

  // 次の順路が到達可能なら次を探す
  index_next = INDEX(_next_x, _next_y, _next_z);
  rt_next = _route_list + index_next;
  if(IS_OPEN(rt_next)) {
    // 既に順路構築不可能な座標の場合は、優先順位を上げる
    object_no = _object_no_list[index_next];
    obj = _object_list + object_no;
    rt_gen = _route_list + INDEX(obj->gen.x, obj->gen.y, obj->gen.z);
    rt_equ = _route_list + INDEX(obj->equ.x, obj->equ.y, obj->equ.z);
    // 次の座標がまだ有効かどうか
    cost = 0;
    if(IS_OPEN(rt_gen) && IS_OPEN(rt_equ)) {
      cost += obj->cost;
    }
    // 順路の紐付け
    index_from = INDEX(_from_x, _from_y, _from_z);
    rt_from = _route_list + index_from;
    rt_from->next = index_next;
    rt_next->prev = index_from;
    // 次の順路探索
    kopt_local_search(_object_list, _object_no_list,
                      _next_x, _next_y, _next_z,
                      _to_x, _to_y, _to_z,
                      _depth + 1, _total_cost + cost,
                      _best_cost, _best_route, _route_list);
    // 順路の紐付け削除
    rt_from->next = -1;
    rt_next->prev = -1;
  }
}

//////////////////////////////
// K-OPT 記録判定と更新
//////////////////////////////
void kopt_best_judge(int const _x, int const _y, int const _z,
                      int const _total_cost, int* const _best_cost,
                      int* const _best_route, route_t* const _route_list) {
  int i, index;

  // 最低コストが見つかった場合は、記録更新
  if(_total_cost < *_best_cost) {
    *_best_cost = _total_cost;
    i = 0;
    index = INDEX(_x, _y, _z);
    while (index != -1) {
      _best_route[i] = index;
      ++ i;
      index = (_route_list + index)->prev;
    }
    _best_route[i] = -1;
  }
}

//////////////////////////////
// 順路の数を取得
//////////////////////////////
int count_route(object_t const* const _object_list, route_t const* const _route_list) {
  int i, count;
  object_t const* obj;
  route_t const* rt;

  count = 0;
  for(i = 0; i < OBJECT_SIZE; ++ i) {
    obj = _object_list + i;
    rt = _route_list + INDEX(obj->gen.x, obj->gen.y, obj->gen.z);;
    if(IS_HEAD(rt)) {
      ++ count;
    }
  }
  return count;
}

//////////////////////////////
// グラフ出力
//////////////////////////////
void plot(object_t const* const _object_list, route_t const* const _route_list) {
  FILE* gp;
  int i, route_index;
  object_t const* obj;
  route_t const* rt;

  gp = popen("gnuplot -persist","w");
  fprintf(gp, "set xrange [-1:20]\n");
  fprintf(gp, "set yrange [-1:20]\n");
  fprintf(gp, "set zrange [-1:20]\n");
  fprintf(gp, "set ticslevel 0\n");
  fprintf(gp, "splot '-' with lines linetype 1 title \"route\"\n");

  for(i = 0; i < OBJECT_SIZE; ++ i) {
    obj = _object_list + i;
    route_index = INDEX(obj->gen.x, obj->gen.y, obj->gen.z);
    rt = _route_list + route_index;
    if(IS_HEAD(rt)) {
      while (route_index != -1) {
        rt = _route_list + route_index;
        fprintf(gp, "%d\t%d\t%d\n", rt->coord.x, rt->coord.y, rt->coord.z);
        route_index = rt->next;
      }
      fprintf(gp,"\n");
    }
  }
  fprintf(gp,"e\n");
  fflush(gp);
  pclose(gp);
}

//////////////////////////////
// 順路をエクスポート
//////////////////////////////
void export_route(object_t const* const _object_list,
                  route_t const* const _route_list, char const* const _file_name) {
  FILE* fp = NULL;
  int i, total, length, route_index;
  object_t const* obj;
  route_t const* rt;

  // ファイルを開く
  if((fp = fopen(_file_name, "w")) == NULL) {
    return;
  }
  // 順路の数を取得
  total = count_route(_object_list, _route_list);
  // 順路数を書き込み
  fprintf(fp, "%d\n", total);
  // 順路内容を書き込み
  for(i = 0; i < OBJECT_SIZE; ++ i) {
    obj = _object_list + i;
    route_index = INDEX(obj->gen.x, obj->gen.y, obj->gen.z);
    rt = _route_list + route_index;
    if(IS_HEAD(rt)) {
      // 順路長を書き込み
      length = 0;
      while (route_index != -1) {
        ++ length;
        rt = _route_list + route_index;
        route_index = rt->next;
      }
      fprintf(fp, "%d\n", length);
      // 順路座標を書き込み
      route_index = INDEX(obj->gen.x, obj->gen.y, obj->gen.z);
      while (route_index != -1) {
        rt = _route_list + route_index;
        fprintf(fp, "%d %d %d\n", rt->coord.x, rt->coord.y, rt->coord.z);
        route_index = rt->next;
      }
    }
  }
  // ファイルを閉じる
  fclose(fp);
}

//////////////////////////////
// 順路ファイルの整合チェック
//////////////////////////////
int route_file_check(int const* const _object_no_list, char const* const _file_name) {
  FILE* fp = NULL;
  int* mem = NULL;
  int alloc_size, total, length;
  int i, j, x, y, z;
  int gen_x, gen_y, gen_z, equ_x, equ_y, equ_z;

  // メモリ確保
  alloc_size = sizeof(int) * SCALE_SIZE * SCALE_SIZE * SCALE_SIZE;
  if((mem = (int*)malloc(alloc_size)) == NULL) {
    return -1;
  }
  // チェックリスト初期化
  memset(mem, 0, alloc_size);
  // ファイルを開く
  if((fp = fopen(_file_name, "r")) == NULL) {
    return -2;
  }
  // 順路数の読み込み
  if(fscanf(fp, "%d", &total) != 1) {
    return -3;
  }
  // 順路チェック
  for(i = 0; i < total; ++ i) {
    if(fscanf(fp, "%d", &length) != 1) {
      return -4;
    }
    gen_x = gen_y = gen_z = -1;
    equ_x = equ_y = equ_z = -1;
    for(j = 0; j < length; ++ j) {
      if(fscanf(fp, "%d %d %d", &x, &y, &z) != 3) {
        return -5;
      }
      // 重複チェック
      if(mem[INDEX(x, y, z)] != 0) {
        return -6;
      }
      mem[INDEX(x, y, z)] = 1;
      if(j == 0) {
        gen_x = x;
        gen_y = y;
        gen_z = z;
      } else if(j == (length - 1)) {
        equ_x = x;
        equ_y = y;
        equ_z = z;
      }
    }
    //開始、終了が同じNo.かチェック
    if(_object_no_list[INDEX(gen_x, gen_y, gen_z)] != _object_no_list[INDEX(equ_x, equ_y, equ_z)]) {
      return -7;
    }
  }
  // データが残っていないかチェック
  if(fscanf(fp, "%d", &total) != EOF) {
    return -8;
  }
  // ファイルを閉じる
  free(mem);
  fclose(fp);
  return 0;
}

static uint32_t xorsft_x = 123456789;
static uint32_t xorsft_y = 362436069;
static uint32_t xorsft_z = 521288629;
static uint32_t xorsft_w = 88675123;

//////////////////////////////
// 乱数seed設定
//////////////////////////////
void initrand(uint32_t seed) {
  do {
      seed = seed*1812433253 + 1; seed ^= seed<<13; seed ^= seed>>17;
      xorsft_x = 123464980 ^ seed;
      seed = seed*1812433253 + 1; seed ^= seed<<13; seed ^= seed>>17;
      xorsft_y = 3447902351 ^ seed;
      seed = seed*1812433253 + 1; seed ^= seed<<13; seed ^= seed>>17;
      xorsft_z = 2859490775 ^ seed;
      seed = seed*1812433253 + 1; seed ^= seed<<13; seed ^= seed>>17;
      xorsft_w = 47621719 ^ seed;
  } while(xorsft_x==0 && xorsft_y==0 && xorsft_z==0 && xorsft_w==0);
}

//////////////////////////////
// 0〜1未満の乱数生成
//////////////////////////////
double urand() {
  uint32_t t;
  t = xorsft_x ^ (xorsft_x<<11);
  xorsft_x = xorsft_y;
  xorsft_y = xorsft_z;
  xorsft_z = xorsft_w;
  xorsft_w ^= t ^ (t>>8) ^ (xorsft_w>>19);
  return ((xorsft_x+0.5) / 4294967296.0 + xorsft_w) / 4294967296.0;
}

//////////////////////////////
// Zoning Remove
//////////////////////////////
void zoning_remove(route_t const* const _route_parent, route_t* const _route_child) {
  int begin_x, end_x, width_x;
  int begin_y, end_y, width_y;
  int begin_z, end_z, width_z;
  int x, y, z;
  int copy_size, route_index;

  // X座標の幅と位置を決める
  width_x = (int)(urand() * (double)(ZX_WIDTH_MAX - ZX_WIDTH_MIN)) + ZX_WIDTH_MIN;
  begin_x = (int)(urand() * (double)(SCALE_SIZE - width_x));
  end_x = begin_x + width_x;
  // Y座標の幅と位置を決める
  width_y = (int)(urand() * (double)(ZX_WIDTH_MAX - ZX_WIDTH_MIN)) + ZX_WIDTH_MIN;
  begin_y = (int)(urand() * (double)(SCALE_SIZE - width_y));
  end_y = begin_y + width_y;
  // Z座標の幅と位置を決める
  width_z = (int)(urand() * (double)(ZX_WIDTH_MAX - ZX_WIDTH_MIN)) + ZX_WIDTH_MIN;
  begin_z = (int)(urand() * (double)(SCALE_SIZE - width_z));
  end_z = begin_z + width_z;
  // 親順路から子順路にコピー
  copy_size = sizeof(route_t) * SCALE_SIZE * SCALE_SIZE * SCALE_SIZE;
  memcpy(_route_child, _route_parent, copy_size);
  // 順路削除
  for(x = begin_x; x <= end_x; ++ x) {
    for(y = begin_y; y <= end_y; ++ y) {
      for(z = begin_z; z <= end_z; ++ z) {
        route_index = INDEX(x, y, z);
        // ゾーン内に接している順路を削除
        zx_remove_route(route_index, _route_child);
      }
    }
  }
}

//////////////////////////////
// Zoning Crossover (ZX)
//////////////////////////////
// void zx_crossover(object_t const* const _object_list, int const* const _object_no_list, qsort_t const* const _cost_list,
//                   route_t const* const _route_parent_1, route_t const* const _route_parent_2,
//                   route_t* const _route_child_1, route_t* const _route_child_2) {
//   int begin_x, end_x, width_x;
//   int begin_y, end_y, width_y;
//   int begin_z, end_z, width_z;
//   int x, y, z;
//   int copy_size, route_index;

//   // X座標の幅と位置を決める
//   width_x = (int)(urand() * (double)(ZX_WIDTH_MAX - ZX_WIDTH_MIN)) + ZX_WIDTH_MIN;
//   begin_x = (int)(urand() * (double)(SCALE_SIZE - width_x));
//   end_x = begin_x + width_x;
//   // Y座標の幅と位置を決める
//   width_y = (int)(urand() * (double)(ZX_WIDTH_MAX - ZX_WIDTH_MIN)) + ZX_WIDTH_MIN;
//   begin_y = (int)(urand() * (double)(SCALE_SIZE - width_y));
//   end_y = begin_y + width_y;
//   // Z座標の幅と位置を決める
//   width_z = (int)(urand() * (double)(ZX_WIDTH_MAX - ZX_WIDTH_MIN)) + ZX_WIDTH_MIN;
//   begin_z = (int)(urand() * (double)(SCALE_SIZE - width_z));
//   end_z = begin_z + width_z;
//   // 親順路から子順路にコピー
//   copy_size = sizeof(route_t) * SCALE_SIZE * SCALE_SIZE * SCALE_SIZE;
//   memcpy(_route_child_1, _route_parent_1, copy_size);
//   memcpy(_route_child_2, _route_parent_2, copy_size);

//   for(x = begin_x; x <= end_x; ++ x) {
//     for(y = begin_y; y <= end_y; ++ y) {
//       for(z = begin_z; z <= end_z; ++ z) {
//         route_index = INDEX(x, y, z);
//         // ゾーン内に接している順路を削除
//         zx_remove_route(route_index, _route_child_1);
//         zx_remove_route(route_index, _route_child_2);
//       }
//     }
//   }
// }

//////////////////////////////
// ゾーンの内側にルートがあるかどうか判定
//////////////////////////////
int zx_inner_zone(int const _begin_x, int const _end_x,
                  int const _begin_y, int const _end_y,
                  int const _begin_z, int const _end_z,
                  int const _route_index, route_t const* const _route_list) {
  int index;
  route_t const* rt;

  // NEXT チェック
  index = _route_index;
  while(index != -1) {
    rt = _route_list + index;
    // はみ出しチェック
    if(rt->coord.x < _begin_x || _end_x < rt->coord.x
    || rt->coord.y < _begin_y || _end_y < rt->coord.y
    || rt->coord.z < _begin_z || _end_z < rt->coord.z) {
      return 0;
    }
    index = rt->next;
  }
  // PREV チェック
  index = _route_index;
  while(index != -1) {
    rt = _route_list + index;
    // はみ出しチェック
    if(rt->coord.x < _begin_x || _end_x < rt->coord.x
    || rt->coord.y < _begin_y || _end_y < rt->coord.y
    || rt->coord.z < _begin_z || _end_z < rt->coord.z) {
      return 0;
    }
    index = rt->prev;
  }
  return 1;
}

//////////////////////////////
// 指定された順路を削除する
//////////////////////////////
void zx_remove_route(int const _route_index, route_t* const _route_list) {
  int index;
  route_t* rt;

  // NEXT 削除
  rt = _route_list + _route_index;
  index = rt->next;
  rt->next = -1;
  while(index != -1) {
    rt = _route_list + index;
    index = rt->next;
    rt->prev = -1;
    rt->next = -1;
  }
  // PREV 削除
  rt = _route_list + _route_index;
  index = rt->prev;
  rt->prev = -1;
  while(index != -1) {
    rt = _route_list + index;
    index = rt->prev;
    rt->prev = -1;
    rt->next = -1;
  }
}