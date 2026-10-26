#!/usr/bin/env python3
"""rosbag_recorder UI用HTTPサーバー。

python3 -m http.server 8000 の置き換え。静的ファイル配信に加えて
乗車人数を passenger_data.json に書き込むAPIを持つ。

  GET  /passengers_today      -> {"date": "...", "am": n|null, "pm": n|null, "slot": "am"|"pm"}
  POST /set_passengers        <- {"count": n, "slot": "am"|"pm"}

バス停ごとの乗車・降車(2026-10-26、運転手の画面の「扉」タブから):
  GET  /stop_passengers_today -> {"date": "...", "slot": "am"|"pm", "records": [...], "am": n|null, "pm": n|null}
  POST /set_stop_passengers   <- {"route": filename, "route_name": .., "stop_index": n, "stop_id": .., "stop_name": ..,
                                  "board": n, "alight": n, "slot": "am"|"pm"(省略可)}
  1 件 1 行で passenger_stops.json に残す。同じ日・便・経路・バス停の番号は後の値で上書きする。
  記録するたびに、その便の乗車の合計を passenger_data.json の am / pm に書く(降車は合計に入れない)。
  画面は ava1 から開くので、CORS を許す。

slot の自動判定は 12:30 を境に am/pm（kururin_web_chart_uploader.py と同じ基準）。
"""

import datetime
import json
import os
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

PORT = 8000
PASSENGER_JSON = Path('/home/sit/tfnh621_workspace/kururin_autonomous_chart_web/passenger_data.json')
STOP_PASSENGER_JSON = PASSENGER_JSON.parent / 'passenger_stops.json'


def current_slot(now=None):
    now = now or datetime.datetime.now()
    return 'am' if now.hour * 60 + now.minute < 12 * 60 + 30 else 'pm'


def write_json(path, obj):
    # 書きかけで落ちても元のファイルを壊さないよう、別名に書いてから置き換える
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(obj, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    os.replace(tmp, path)


def load_records():
    if PASSENGER_JSON.exists():
        return json.loads(PASSENGER_JSON.read_text(encoding='utf-8'))
    return []


def save_records(records):
    records.sort(key=lambda r: r['date'])
    write_json(PASSENGER_JSON, records)


def today_record(records, today):
    for r in records:
        if r['date'] == today:
            return r
    r = {'date': today, 'am': None, 'pm': None}
    records.append(r)
    return r


def load_stop_records():
    if STOP_PASSENGER_JSON.exists():
        return json.loads(STOP_PASSENGER_JSON.read_text(encoding='utf-8'))
    return []


def upsert_stop_record(stop_records, rec):
    """同じ日・便・経路・バス停の番号の行があれば置き換え、なければ足す。"""
    key = (rec['date'], rec['slot'], rec['route'], rec['stop_index'])
    stop_records[:] = [r for r in stop_records if (r['date'], r['slot'], r['route'], r['stop_index']) != key]
    stop_records.append(rec)
    stop_records.sort(key=lambda r: (r['date'], r['time']))


def board_total(stop_records, date, slot):
    rows = [r for r in stop_records if r['date'] == date and r['slot'] == slot]
    return sum(r['board'] for r in rows) if rows else None


class Handler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def send_json(self, obj, status=200):
        body = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def read_json(self):
        length = int(self.headers.get('Content-Length', 0))
        return json.loads(self.rfile.read(length))

    def stop_passengers_today(self):
        today = datetime.date.today().isoformat()
        stop_records = load_stop_records()
        return {'date': today, 'slot': current_slot(),
                'records': [r for r in stop_records if r['date'] == today],
                'am': board_total(stop_records, today, 'am'), 'pm': board_total(stop_records, today, 'pm')}

    def do_GET(self):
        path = self.path.split('?')[0]
        if path == '/passengers_today':
            today = datetime.date.today().isoformat()
            record = next((r for r in load_records() if r['date'] == today),
                          {'date': today, 'am': None, 'pm': None})
            record = dict(record, slot=current_slot())
            self.send_json(record)
            return
        if path == '/stop_passengers_today':
            self.send_json(self.stop_passengers_today())
            return
        super().do_GET()

    def do_POST(self):
        if self.path == '/set_stop_passengers':
            self.set_stop_passengers()
            return
        if self.path != '/set_passengers':
            self.send_json({'ok': False, 'error': 'unknown endpoint'}, status=404)
            return
        try:
            req = self.read_json()
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

    def set_stop_passengers(self):
        now = datetime.datetime.now()
        try:
            req = self.read_json()
            rec = {
                'date': now.date().isoformat(),
                'time': now.strftime('%H:%M:%S'),
                'slot': req.get('slot') or current_slot(now),
                'route': str(req['route']),
                'route_name': str(req.get('route_name', '')),
                'stop_index': int(req['stop_index']),
                'stop_id': str(req.get('stop_id', '')),
                'stop_name': str(req.get('stop_name', '')),
                'board': int(req['board']),
                'alight': int(req['alight']),
            }
            if rec['slot'] not in ('am', 'pm') or rec['board'] < 0 or rec['alight'] < 0:
                raise ValueError(f"invalid slot/board/alight: {rec['slot']}/{rec['board']}/{rec['alight']}")
        except (ValueError, KeyError, TypeError, json.JSONDecodeError) as e:
            self.send_json({'ok': False, 'error': str(e)}, status=400)
            return

        stop_records = load_stop_records()
        upsert_stop_record(stop_records, rec)
        write_json(STOP_PASSENGER_JSON, stop_records)

        total = board_total(stop_records, rec['date'], rec['slot'])
        records = load_records()
        today_record(records, rec['date'])[rec['slot']] = total
        save_records(records)
        self.send_json(dict(self.stop_passengers_today(), ok=True, saved=rec))


if __name__ == '__main__':
    print(f'Serving on port {PORT}, passenger json: {PASSENGER_JSON}, per-stop: {STOP_PASSENGER_JSON}')
    HTTPServer(('', PORT), Handler).serve_forever()
