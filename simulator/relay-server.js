// ==================================================================
//  relay-server.js
//  Server relay: menghubungkan Remote Control (dibuka di HP)
//  dengan Simulator 3D Monocopter (dibuka di laptop) lewat WebSocket,
//  MENGGANTIKAN peran ESP32 sebagai WebSocket server.
//
//  (RC & simulator)
//  sama-sama membuka koneksi ke server relay ini, lalu server ini
//  meneruskan (broadcast) setiap pesan ke SEMUA client lain:
//    HP (RC)       --{cmd:"rc",...}-->      relay  --> Simulator (laptop)
//    Simulator     --{att,alt,sys,...}-->   relay  --> HP (RC, telemetry)
//
//  CARA PAKAI
//  ----------
//  1) Pastikan Node.js sudah terpasang (https://nodejs.org).
//  2) Taruh file ini SATU FOLDER dengan:
//       - rc.html, rc.js   (remote control)
//       - simulator.html
//  3) Buka terminal di folder tsb, jalankan sekali saja:
//       npm install ws
//  4) Jalankan server:
//       node relay-server.js
//  5) Di LAPTOP, buka simulator lewat:
//       http://localhost:8090/simulator.html
//  6) Di HP (WAJIB satu jaringan WiFi yang sama dengan laptop), buka:
//       http://<IP-LAPTOP-DI-TERMINAL>:8090/rc.html
//     (alamat IP laptop akan otomatis dicetak di terminal setelah
//     server dijalankan - tinggal salin ke browser HP)
// ==================================================================

const http = require('http');
const fs   = require('fs');
const path = require('path');
const os   = require('os');

let WebSocket;
try {
  WebSocket = require('ws');
} catch (e) {
  console.error('\n[ERROR] Modul "ws" belum terpasang.');
  console.error('Jalankan dulu:  npm install ws\n');
  process.exit(1);
}

const PORT = 8090;
const ROOT = __dirname;

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js':   'text/javascript; charset=utf-8',
  '.css':  'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.png':  'image/png',
  '.jpg':  'image/jpeg',
  '.svg':  'image/svg+xml',
};

// --- HTTP server: menyajikan file statis (RC & simulator) ---
const server = http.createServer((req, res) => {
  let reqPath = decodeURIComponent((req.url || '/').split('?')[0]);
  if (reqPath === '/') reqPath = '/rc.html';
  const filePath = path.normalize(path.join(ROOT, reqPath));
  const clientIp = req.socket.remoteAddress;
  if (!filePath.startsWith(ROOT)) {
    console.log(`[http] DITOLAK (403) dari ${clientIp} - path: ${reqPath}`);
    res.writeHead(403); res.end('Forbidden'); return;
  }
  fs.readFile(filePath, (err, data) => {
    if (err) {
      console.log(`[http] 404 dari ${clientIp} - file tidak ada: ${reqPath}`);
      res.writeHead(404); res.end('404 - file tidak ditemukan: ' + reqPath); return;
    }
    console.log(`[http] 200 OK dari ${clientIp} - ${reqPath}`);
    const ext = path.extname(filePath);
    res.writeHead(200, { 'Content-Type': MIME[ext] || 'application/octet-stream' });
    res.end(data);
  });
});

// --- WebSocket relay: broadcast pesan ke semua client LAIN ---
const wss = new WebSocket.Server({ server, path: '/ws' });

wss.on('connection', (ws, req) => {
  const ip = req.socket.remoteAddress;
  console.log(`[relay] client masuk (${ip}) - total client: ${wss.clients.size}`);

  ws.on('message', (msg) => {
    wss.clients.forEach((client) => {
      if (client !== ws && client.readyState === WebSocket.OPEN) {
        client.send(msg.toString());
      }
    });
  });

  ws.on('close', () => {
    console.log(`[relay] client keluar (${ip}) - total client: ${wss.clients.size}`);
  });

  ws.on('error', (e) => console.log('[relay] ws error:', e.message));
});

// --- Start ---
server.listen(PORT, () => {
  const nets = os.networkInterfaces();
  const ips = [];
  for (const name of Object.keys(nets)) {
    for (const net of nets[name]) {
      if (net.family === 'IPv4' && !net.internal) ips.push(net.address);
    }
  }
  console.log('========================================================');
  console.log(' Monocopter RC <-> Simulator relay AKTIF - port', PORT);
  console.log('========================================================');
  console.log(' [LAPTOP] Buka simulator 3D di browser laptop ini:');
  console.log('   http://localhost:' + PORT + '/simulator.html');
  console.log('');
  console.log(' [HP] Buka remote control di HP (WAJIB WiFi SAMA dgn laptop):');
  if (ips.length) {
    ips.forEach((ip) => console.log('   http://' + ip + ':' + PORT + '/rc.html'));
  } else {
    console.log('   (Tidak terdeteksi IP LAN. Pastikan laptop tersambung WiFi/Ethernet,');
    console.log('    lalu cek IP manual lewat "ipconfig" [Windows] / "ifconfig" [Mac/Linux].)');
  }
  console.log('========================================================');
  console.log(' Tekan Ctrl+C untuk menghentikan server.');
  console.log('========================================================');
});
