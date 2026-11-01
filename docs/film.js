// Play the desk film (muted) while it is on screen; pause it when it scrolls away.
// Someone who paused or unmuted it themselves keeps control.
const film = document.querySelector("#desk video");
if (film && "IntersectionObserver" in window) {
  let userPaused = false;
  film.addEventListener("pause", () => { if (film.dataset.auto !== "pause") userPaused = true; delete film.dataset.auto; });
  film.addEventListener("play", () => { userPaused = false; });
  new IntersectionObserver(([entry]) => {
    if (entry.isIntersecting && film.paused && !userPaused) {
      film.muted = true;
      film.play().catch(() => {});
    } else if (!entry.isIntersecting && !film.paused) {
      film.dataset.auto = "pause";
      film.pause();
    }
  }, { threshold: 0.5 }).observe(film);
}
