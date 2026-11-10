// Short sound, vibration and spoken replies, so the user knows the robot heard them
// without looking at the screen.

let audio;

export function beep(freq = 880, ms = 120) {
  try {
    audio ??= new AudioContext();
    const osc = audio.createOscillator();
    const gain = audio.createGain();
    osc.frequency.value = freq;
    gain.gain.value = 0.15;
    osc.connect(gain).connect(audio.destination);
    osc.start();
    osc.stop(audio.currentTime + ms / 1000);
  } catch {
    // no audio output; the visual state still changes
  }
  navigator.vibrate?.(ms);
}

/** Speak Thai text; resolves when finished (or at once if the browser cannot speak). */
export function say(text) {
  return new Promise((resolve) => {
    if (!('speechSynthesis' in window)) return resolve();
    const u = new SpeechSynthesisUtterance(text);
    u.lang = 'th-TH';
    u.voice = speechSynthesis.getVoices().find((v) => v.lang.startsWith('th')) ?? null;
    u.onend = u.onerror = () => resolve();
    speechSynthesis.cancel();
    speechSynthesis.speak(u);
    setTimeout(resolve, 6000); // some browsers never fire onend
  });
}
