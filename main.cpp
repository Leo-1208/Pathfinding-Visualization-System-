#include <bits/stdc++.h>
using namespace std;

using ll = long long;
const ll INF = (1LL<<60);

// ── grid + helpers ─────────────────────────────────────────────────────────────
int H, W, S = -1, G = -1;
vector<string> grid;

inline int id(int r,int c){ return r*W + c; }
inline pair<int,int> rc(int v){ return {v/W, v%W}; }
inline bool inb(int r,int c){ return 0<=r && r<H && 0<=c && c<W; }
inline bool wall(int r,int c){ return grid[r][c] == '#'; }

inline int cellCost(int r,int c){
    char ch = grid[r][c];
    if(ch=='#') return 1e9;
    if(ch=='.' || ch=='S' || ch=='G') return 1;
    if('1'<=ch && ch<='9') return ch - '0';
    return 1;
}

// ── path & stats ───────────────────────────────────────────────────────────────
struct Stat{
    bool ok = false;
    long long micros = 0;
    long long explored = 0;
    vector<int> path, explored_nodes;
    ll cost = INF;
};

vector<int> buildPath(const vector<int>& par, int s, int t){
    vector<int> p;
    if(s<0 || t<0) return p;
    if(par[t]==-1 && t!=s) return p;
    for(int cur=t; cur!=-1; cur=par[cur]) p.push_back(cur);
    reverse(p.begin(), p.end());
    if(p.empty() || p.front()!=s) p.clear();
    return p;
}

ll pathCost(const vector<int>& p){
    if(p.empty()) return INF;
    ll ans = 0;
    for(size_t i=1;i<p.size();++i){
        auto [r,c] = rc(p[i]);
        ans += cellCost(r,c);
    }
    return ans;
}

// ── dump explored + final to file ──────────────────────────────────────────────
void dumpMarked(ostream& os, const string& title,
                const vector<int>& path, const vector<int>& explored){
    vector<char> mark(H*W, 0); // 0 none, 1 explored, 2 path
    for(int v : explored) if(0<=v && v<H*W) mark[v] = max<char>(mark[v], 1);
    for(int v : path)     if(0<=v && v<H*W) mark[v] = 2;

    os << "\n=== " << title << " (original) ===\n";
    for(auto& row: grid) os << row << "\n";

    os << "\n=== " << title << " (marks only) ===\n";
    for(int r=0;r<H;r++){
        for(int c=0;c<W;c++){
            if(grid[r][c]=='#'){ os << '#'; continue; }
            int v = id(r,c);
            os << (mark[v]==2 ? '*'
                 : mark[v]==1 ? '+'
                               : '.');
        }
        os << "\n";
    }
}

// ── BFS ────────────────────────────────────────────────────────────────────────
Stat BFS(){
    Stat st;
    auto t0 = chrono::high_resolution_clock::now();

    vector<int> dist(H*W, -1), par(H*W, -1);
    queue<int> q; q.push(S); dist[S]=0;
    vector<int> explored;
    int dr[4]={1,-1,0,0}, dc[4]={0,0,1,-1};

    while(!q.empty()){
        int u=q.front(); q.pop(); explored.push_back(u);
        if(u==G) break;
        auto [r,c]=rc(u);
        for(int k=0;k<4;k++){
            int nr=r+dr[k], nc=c+dc[k];
            if(!inb(nr,nc) || wall(nr,nc)) continue;
            int v=id(nr,nc);
            if(dist[v]==-1){ dist[v]=dist[u]+1; par[v]=u; q.push(v); }
        }
    }
    st.path = buildPath(par, S, G);
    st.ok   = !st.path.empty();
    st.explored = (long long)explored.size();
    st.explored_nodes = move(explored);
    st.cost = st.ok ? pathCost(st.path) : INF;
    st.micros = chrono::duration_cast<chrono::microseconds>(
        chrono::high_resolution_clock::now()-t0).count();
    return st;
}

// ── DFS ────────────────────────────────────────────────────────────────────────
Stat DFS(){
    Stat st;
    auto t0 = chrono::high_resolution_clock::now();

    vector<char> vis(H*W,0); vector<int> par(H*W,-1);
    stack<int> stck; stck.push(S); vis[S]=1;
    vector<int> explored;
    int dr[4]={1,-1,0,0}, dc[4]={0,0,1,-1};

    while(!stck.empty()){
        int u=stck.top(); stck.pop(); explored.push_back(u);
        if(u==G) break;
        auto [r,c]=rc(u);
        for(int k=0;k<4;k++){
            int nr=r+dr[k], nc=c+dc[k];
            if(!inb(nr,nc) || wall(nr,nc)) continue;
            int v=id(nr,nc);
            if(!vis[v]){ vis[v]=1; par[v]=u; stck.push(v); }
        }
    }
    st.path = buildPath(par, S, G);
    st.ok   = (S==G) || !st.path.empty();
    st.explored = (long long)explored.size();
    st.explored_nodes = move(explored);
    st.cost = st.ok ? pathCost(st.path) : INF; // not optimal in general
    st.micros = chrono::duration_cast<chrono::microseconds>(
        chrono::high_resolution_clock::now()-t0).count();
    return st;
}

// ── Dijkstra ───────────────────────────────────────────────────────────────────
Stat Dijkstra(){
    Stat st;
    auto t0 = chrono::high_resolution_clock::now();

    vector<ll> dist(H*W, INF); vector<int> par(H*W,-1);
    vector<char> done(H*W, 0);
    using P = pair<ll,int>;
    priority_queue<P, vector<P>, greater<P>> pq;
    dist[S]=0; pq.push({0,S});
    vector<int> explored;
    int dr[4]={1,-1,0,0}, dc[4]={0,0,1,-1};

    while(!pq.empty()){
        auto [d,u]=pq.top(); pq.pop();
        if(done[u]) continue; done[u]=1; explored.push_back(u);
        if(u==G) break;
        auto [r,c]=rc(u);
        for(int k=0;k<4;k++){
            int nr=r+dr[k], nc=c+dc[k];
            if(!inb(nr,nc) || wall(nr,nc)) continue;
            int v=id(nr,nc), w=cellCost(nr,nc);
            if(dist[v] > d + w){
                dist[v]=d+w; par[v]=u; pq.push({dist[v], v});
            }
        }
    }
    st.ok = (dist[G] < INF);
    st.path = buildPath(par, S, G);
    st.explored = (long long)explored.size();
    st.explored_nodes = move(explored);
    st.cost = st.ok ? pathCost(st.path) : INF;
    st.micros = chrono::duration_cast<chrono::microseconds>(
        chrono::high_resolution_clock::now()-t0).count();
    return st;
}

// ── Greedy Best-First (h only) ────────────────────────────────────────────────
inline int hManhattan(int u,int v){
    auto [r1,c1]=rc(u); auto [r2,c2]=rc(v);
    return abs(r1-r2) + abs(c1-c2);
}

Stat GreedyBF(){
    Stat st;
    auto t0 = chrono::high_resolution_clock::now();

    vector<char> closed(H*W,0);
    vector<int> par(H*W,-1);
    using P = pair<int,int>; // h, node
    priority_queue<P, vector<P>, greater<P>> open;
    open.push({hManhattan(S,G), S});
    vector<int> explored;
    int dr[4]={1,-1,0,0}, dc[4]={0,0,1,-1};

    while(!open.empty()){
        auto [h,u]=open.top(); open.pop();
        if(closed[u]) continue; closed[u]=1; explored.push_back(u);
        if(u==G) break;
        auto [r,c]=rc(u);
        for(int k=0;k<4;k++){
            int nr=r+dr[k], nc=c+dc[k];
            if(!inb(nr,nc) || wall(nr,nc)) continue;
            int v=id(nr,nc);
            if(!closed[v]){
                if(par[v]==-1) par[v]=u;
                open.push({hManhattan(v,G), v});
            }
        }
    }
    st.ok = closed[G];
    st.path = buildPath(par, S, G);
    st.explored = (long long)explored.size();
    st.explored_nodes = move(explored);
    st.cost = st.ok ? pathCost(st.path) : INF;
    st.micros = chrono::duration_cast<chrono::microseconds>(
        chrono::high_resolution_clock::now()-t0).count();
    return st;
}

// ── A* ────────────────────────────────────────────────────────────────────────
Stat AStar(){
    Stat st;
    auto t0 = chrono::high_resolution_clock::now();

    vector<ll> g(H*W, INF);
    vector<int> par(H*W, -1);
    vector<char> closed(H*W, 0);
    using P = pair<ll,int>; // f, node
    priority_queue<P, vector<P>, greater<P>> open;
    g[S]=0; open.push({(ll)hManhattan(S,G), S});
    vector<int> explored;
    int dr[4]={1,-1,0,0}, dc[4]={0,0,1,-1};

    while(!open.empty()){
        auto [f,u]=open.top(); open.pop();
        if(closed[u]) continue; closed[u]=1; explored.push_back(u);
        if(u==G) break;
        auto [r,c]=rc(u);
        for(int k=0;k<4;k++){
            int nr=r+dr[k], nc=c+dc[k];
            if(!inb(nr,nc) || wall(nr,nc)) continue;
            int v=id(nr,nc);
            ll w = cellCost(nr,nc);
            if(g[v] > g[u] + w){
                g[v] = g[u] + w; par[v]=u;
                ll f2 = g[v] + hManhattan(v,G);
                open.push({f2, v});
            }
        }
    }
    st.ok = (g[G] < INF);
    st.path = buildPath(par, S, G);
    st.explored = (long long)explored.size();
    st.explored_nodes = move(explored);
    st.cost = st.ok ? pathCost(st.path) : INF;
    st.micros = chrono::duration_cast<chrono::microseconds>(
        chrono::high_resolution_clock::now()-t0).count();
    return st;
}

// ── Bellman-Ford ──────────────────────────────────────────────────────────────
Stat BellmanFord(){
    Stat st;
    auto t0 = chrono::high_resolution_clock::now();

    struct E{int u,v,w;};
    vector<E> edges; edges.reserve(H*W*4);
    int dr[4]={1,-1,0,0}, dc[4]={0,0,1,-1};
    for(int r=0;r<H;r++) for(int c=0;c<W;c++){
        if(wall(r,c)) continue;
        int u=id(r,c);
        for(int k=0;k<4;k++){
            int nr=r+dr[k], nc=c+dc[k];
            if(inb(nr,nc) && !wall(nr,nc)){
                edges.push_back({u, id(nr,nc), cellCost(nr,nc)});
            }
        }
    }

    vector<ll> dist(H*W, INF); vector<int> par(H*W,-1);
    dist[S]=0;
    for(int i=0;i<H*W-1;i++){
        bool any=false;
        for(auto &e: edges){
            if(dist[e.u]==INF) continue;
            if(dist[e.v] > dist[e.u] + e.w){
                dist[e.v] = dist[e.u] + e.w;
                par[e.v] = e.u; any=true;
            }
        }
        if(!any) break;
    }
    vector<int> explored;
    for(int v=0; v<H*W; v++) if(dist[v]<INF) explored.push_back(v);

    st.ok = (dist[G] < INF);
    st.path = buildPath(par, S, G);
    st.explored = (long long)explored.size();
    st.explored_nodes = move(explored);
    st.cost = st.ok ? pathCost(st.path) : INF;
    st.micros = chrono::duration_cast<chrono::microseconds>(
        chrono::high_resolution_clock::now()-t0).count();
    return st;
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(){
    // read input.txt
    ifstream fin("input.txt");
    if(!fin){ cerr<<"Cannot open input.txt\n"; return 1; }
    if(!(fin>>H>>W)){ cerr<<"Failed to read H W\n"; return 1; }

    grid.assign(H, "");
    for(int r=0;r<H;r++){
        fin>>grid[r];
        if((int)grid[r].size()!=W){ cerr<<"Row width mismatch\n"; return 1; }
        for(int c=0;c<W;c++){
            if(grid[r][c]=='S') S=id(r,c);
            if(grid[r][c]=='G') G=id(r,c);
        }
    }
    if(S==-1 || G==-1){ cerr<<"Need S and G\n"; return 1; }

    // run
    auto B  = BFS();
    auto Df = DFS();
    auto Dj = Dijkstra();
    auto Gb = GreedyBF();
    auto A  = AStar();
    auto Bf = BellmanFord();

    // console table
    cout<<left<<setw(18)<<"Algorithm"
        <<setw(8)<<"Found"
        <<setw(16)<<"NodesExplored"
        <<setw(12)<<"Time(µs)"
        <<"Cost\n";
    auto row=[&](string n, const Stat& s){
        cout<<left<<setw(18)<<n
            <<setw(8)<<(s.ok?"YES":"NO")
            <<setw(16)<<s.explored
            <<setw(12)<<s.micros
            <<(s.ok?to_string(s.cost):"N/A")<<"\n";
    };
    row("BFS",             B);
    row("DFS",             Df);
    row("Dijkstra",        Dj);
    row("GreedyBestFirst", Gb);
    row("A*",              A);
    row("Bellman-Ford",    Bf);

    // output.txt marks
    ofstream fout("output.txt");
    if(!fout){ cerr<<"Cannot open output.txt\n"; return 1; }
    auto dump=[&](const string& n, const Stat& s){
        if(!s.ok){ fout << "\n=== " << n << " ===\nNo path found.\n"; }
        else{
            dumpMarked(fout, n, s.path, s.explored_nodes);
            fout << "Path cost: " << s.cost
                 << ", Steps: " << (int)s.path.size() << "\n";
        }
    };
    dump("BFS (unweighted shortest path)", B);
    dump("DFS (any path)",                 Df);
    dump("Dijkstra (weighted)",            Dj);
    dump("Greedy Best-First (h only)",     Gb);
    dump("A* (weighted + admissible h)",   A);
    dump("Bellman-Ford (weighted)",        Bf);

    // best summary
    vector<pair<string,Stat>> all={
        {"BFS",B},{"DFS",Df},{"Dijkstra",Dj},{"Greedy",Gb},{"A*",A},{"Bellman-Ford",Bf}
    };
    ll best=INF; string bestName;
    for(auto &p: all) if(p.second.ok && p.second.cost<best){
        best=p.second.cost; bestName=p.first;
    }
    if(!bestName.empty()) cout<<"\nBest: "<<bestName<<" (Cost="<<best<<")\n";
    else cout<<"\nNo path found by any algorithm.\n";

    return 0;
}
