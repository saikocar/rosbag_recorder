#!/usr/bin/env python3
"""rosbag_recorder UI用HTTPサーバー。

python3 -m http.server 8000 の置き換え。静的ファイル配信に加えて
乗車人数を passenger_data.json に書き込むAPIを持つ。

  GET  /passengers_today      -> {"date": "...", "am": n|null, "pm": n|null, "slot": "am"|"pm"}
  POST /set_passengers        <- {"count": n, "slot": "am"|"pm"}

slot の自動判定は 12:30 を境に am/pm（kururin_web_chart_uploader.py と同じ基準）。
"""

import datetime
import json
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

PORT = 8000
PASSENGER_JSON = Path('/home/sit/tfnh621_workspace/kururin_autonomous_chart_web/passenger_data.json')


def current_slot(now=None):
    now = now or datetime.datetime.now()
    return 'am' if now.hour * 60 + now.minute < 12 * 60 + 30 else 'pm'


def load_records():
    if PASSENGER_JSON.exists():
        return json.loads(PASSENGER_JSON.read_text(encoding='utf-8'))
    return []


def save_records(records):
    records.sort(key=lambda r: r['date'])
    PASSENGER_JSON.write_text(
        json.dumps(records, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def today_record(records, today):
    for r in records:
        if r['date'] == today:
            return r
    r = {'date': today, 'am': None, 'pm': None}
    records.append(r)
    return r


class Handler(SimpleHTTPRequestHandler):
    def send_json(self, obj, status=200):
        body = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.split('?')[0] == '/passengers_today':
            today = datetime.date.today().isoformat()
            record = next((r for r in load_records() if r['date'] == today),
                          {'date': today, 'am': None, 'pm': None})
            record = dict(record, slot=current_slot())
            self.send_json(record)
            return
        super().do_GET()

    def do_POST(self):
        if self.path != '/set_passengers':
            self.send_json({'ok': False, 'error': 'unknown endpoint'}, status=404)
            return
        try:
            length = int(self.headers.get('Content-Length', 0))
            req = json.loads(self.rfile.read(length))
            count = int(req['count'])
            slot = req.get('slot') or current_slot()
            if slot not in ('am', 'pm') or count < 0:
                raise ValueError(f'invalid slot/count: {slot}/{count}')
        except (ValueError, KeyError, json.JSONDecodeError) as e:
            self.send_json({'ok': False, 'error': str(e)}, status=400)
            return

        today = datetime.date.today().isoformat()
        records = load_records()
        today_record(records, today)[slot] = count
        save_records(records)
        self.send_json({'ok': True, 'date': today, 'slot': slot, 'count': count})


if __name__ == '__main__':
    print(f'Serving on port {PORT}, passenger json: {PASSENGER_JSON}')
    HTTPServer(('', PORT), Handler).serve_forever()
