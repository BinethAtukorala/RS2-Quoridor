// QuorZero portfolio — scroll-reveal + mobile nav

(function () {
  'use strict';

  // ----- Scroll-reveal via IntersectionObserver -----
  const reveals = document.querySelectorAll('.reveal');
  if ('IntersectionObserver' in window) {
    const io = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            entry.target.classList.add('is-visible');
            io.unobserve(entry.target);
          }
        });
      },
      { threshold: 0.12, rootMargin: '0px 0px -40px 0px' }
    );
    reveals.forEach((el) => io.observe(el));
  } else {
    reveals.forEach((el) => el.classList.add('is-visible'));
  }

  // ----- Bulma-style burger menu toggle -----
  document.querySelectorAll('.navbar-burger').forEach((burger) => {
    burger.addEventListener('click', () => {
      const target = document.getElementById(burger.dataset.target);
      burger.classList.toggle('is-active');
      if (target) target.classList.toggle('is-active');
    });
  });

  // ----- Lightbox for .board-figure images -----
  const lightbox = document.createElement('div');
  lightbox.className = 'lightbox';
  lightbox.innerHTML = '<button class="lightbox-close" aria-label="Close">✕</button><img alt="">';
  document.body.appendChild(lightbox);
  const lbImg = lightbox.querySelector('img');

  const openLightbox = (src, alt) => {
    lbImg.src = src;
    lbImg.alt = alt || '';
    lightbox.classList.add('is-open');
    document.body.style.overflow = 'hidden';
  };
  const closeLightbox = () => {
    lightbox.classList.remove('is-open');
    document.body.style.overflow = '';
  };

  document.querySelectorAll('.board-figure img').forEach((img) => {
    img.addEventListener('click', () => openLightbox(img.src, img.alt));
  });
  lightbox.addEventListener('click', closeLightbox);
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && lightbox.classList.contains('is-open')) closeLightbox();
  });

  // Collapse mobile menu after clicking a link
  document.querySelectorAll('.navbar-end .navbar-item').forEach((item) => {
    item.addEventListener('click', () => {
      const menu = document.querySelector('.navbar-menu');
      const burger = document.querySelector('.navbar-burger');
      if (menu && menu.classList.contains('is-active')) {
        menu.classList.remove('is-active');
        if (burger) burger.classList.remove('is-active');
      }
    });
  });
})();
