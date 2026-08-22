const DEFAULT_PAGE = "flash";
const pageTitles = {
  flash: "書き込み",
  debug: "デバッグ",
  setup: "Windows 初回設定",
};

const panels = new Map(
  [...document.querySelectorAll("[data-page-panel]")].map((panel) => [
    panel.dataset.pagePanel,
    panel,
  ]),
);

const links = [...document.querySelectorAll("[data-page-link]")];

function pageFromHash() {
  const requested = location.hash.replace(/^#/, "").trim();
  return panels.has(requested) ? requested : DEFAULT_PAGE;
}

function showPage(page, { resetScroll = true } = {}) {
  const selected = panels.has(page) ? page : DEFAULT_PAGE;

  for (const [name, panel] of panels) {
    panel.hidden = name !== selected;
  }

  for (const link of links) {
    const active = link.dataset.pageLink === selected;
    if (active) {
      link.setAttribute("aria-current", "page");
    } else {
      link.removeAttribute("aria-current");
    }
  }

  document.title = `${pageTitles[selected]} - SPIKE-RT Web Toolkit`;

  if (resetScroll) {
    window.scrollTo(0, 0);
  }
}

function syncFromLocation({ resetScroll = true } = {}) {
  const selected = pageFromHash();
  if (location.hash !== `#${selected}`) {
    history.replaceState(null, "", `#${selected}`);
  }
  showPage(selected, { resetScroll });
}

for (const link of links) {
  link.addEventListener("click", () => {
    if (link.dataset.pageLink === pageFromHash()) {
      window.scrollTo(0, 0);
    }
  });
}

window.addEventListener("hashchange", () => syncFromLocation());
syncFromLocation({ resetScroll: false });
