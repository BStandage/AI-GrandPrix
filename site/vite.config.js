import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// Served at https://<user>.github.io/AI-GrandPrix/
export default defineConfig({
  plugins: [react()],
  base: '/AI-GrandPrix/',
})
