// "Hey Armbot" voice command state machine, using the browser's Web Speech API (th-TH).
//
//   standby --wake word--> active (beep + "ค่ะ มีอะไรให้ช่วยคะ", listen up to COMMAND_MS)
//   active --speech--> confirm (show text, match an item, queue it, speak the reply) --> standby
//   active --silence--> standby
//
// Chrome sends the audio to Google for recognition, so it needs internet. Microphone use
// needs HTTPS (or localhost), and the page must stay open: a web page cannot listen in
// the background.

import { beep, say } from './feedback.js';

const WAKE_WORDS = ['hey armbot', 'hey arm bot', 'เฮ้ อาร์มบอท', 'เฮ้อาร์มบอท', 'อาร์มบอท', 'อาร์ม บอท', 'armbot'];
const COMMAND_MS = 8000;

const Recognition = typeof window !== 'undefined' && (window.SpeechRecognition || window.webkitSpeechRecognition);

function afterWake(text) {
  const lower = text.toLowerCase();
  for (const w of WAKE_WORDS) {
    const i = lower.indexOf(w);
    if (i >= 0) return text.slice(i + w.length).trim();
  }
  return null;
}

export function createVoice({ findItem, order }) {
  const v = $state({
    supported: !!Recognition,
    enabled: false,
    mode: 'off', // off | standby | active | confirm
    heard: '', // live transcript
    message: 'กดปุ่มไมค์เพื่อเปิดการสั่งด้วยเสียง',
    error: '',
  });
  let rec;
  let timer;
  let speaking = false;

  function setMode(mode, message) {
    v.mode = mode;
    if (message) v.message = message;
    clearTimeout(timer);
    if (mode === 'active') {
      timer = setTimeout(() => setMode('standby', 'ไม่ได้ยินคำสั่ง พูด "Hey Armbot" อีกครั้ง'), COMMAND_MS);
    }
  }

  async function speak(text) {
    speaking = true;
    rec?.abort(); // do not hear our own voice
    await say(text);
    speaking = false;
    if (v.enabled) start();
  }

  async function handleCommand(text) {
    setMode('confirm', `ได้ยิน: "${text}"`);
    const item = findItem(text);
    if (!item) {
      await speak('ไม่พบของชิ้นนี้ค่ะ');
      setMode('standby', `ไม่พบของในคำสั่ง "${text}"`);
      return;
    }
    try {
      await order({ item_id: item.id, source: 'voice' });
      await speak(`กำลังหยิบ${item.name_th}ค่ะ`);
      setMode('standby', `เพิ่ม ${item.name_th} เข้าคิวแล้ว`);
    } catch (e) {
      await speak('สั่งงานไม่สำเร็จค่ะ');
      setMode('standby', `ผิดพลาด: ${e.message}`);
    }
  }

  async function onText(text, isFinal) {
    v.heard = text;
    if (v.mode === 'standby') {
      const rest = afterWake(text);
      if (rest === null) return;
      beep();
      if (isFinal && rest && findItem(rest)) return handleCommand(rest); // "Hey Armbot หยิบรีเลย์" in one go
      setMode('active', 'กำลังฟังคำสั่ง...');
      await speak('ค่ะ มีอะไรให้ช่วยคะ');
      setMode('active', 'กำลังฟังคำสั่ง...');
    } else if (v.mode === 'active' && isFinal && text.trim()) {
      await handleCommand(text.trim());
    }
  }

  function start() {
    if (!Recognition || speaking) return;
    rec = new Recognition();
    rec.lang = 'th-TH';
    rec.continuous = true;
    rec.interimResults = true;
    rec.onresult = (e) => {
      const r = e.results[e.results.length - 1];
      onText(r[0].transcript, r.isFinal);
    };
    rec.onerror = (e) => {
      if (e.error === 'not-allowed' || e.error === 'service-not-allowed') {
        v.error = 'ไม่ได้รับอนุญาตให้ใช้ไมค์ (ต้องเปิดผ่าน HTTPS)';
        stop();
      } else if (e.error === 'network') {
        v.error = 'ระบบรู้จำเสียงต้องใช้อินเทอร์เน็ต';
      }
    };
    // Chrome ends recognition after a pause; keep listening while enabled.
    rec.onend = () => v.enabled && !speaking && setTimeout(start, 250);
    try {
      rec.start();
    } catch {
      // already started
    }
  }

  function enable() {
    if (!Recognition) return;
    v.enabled = true;
    v.error = '';
    setMode('standby', 'พูด "Hey Armbot" เพื่อสั่งงาน');
    start();
  }

  function stop() {
    v.enabled = false;
    clearTimeout(timer);
    rec?.abort();
    setMode('off', 'การสั่งด้วยเสียงปิดอยู่');
  }

  return { state: v, enable, stop, toggle: () => (v.enabled ? stop() : enable()) };
}
