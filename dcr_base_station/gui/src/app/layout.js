'use client';

import "./globals.css";
import { ThemeProvider } from '@mui/material/styles';
import CssBaseline from '@mui/material/CssBaseline';
import { ROSProvider } from '@/context/ROSContext';
import theme from '@/styles/theme';
import { Rajdhani, Roboto_Mono } from 'next/font/google';

const rajdhani = Rajdhani({
  weight: ['300', '400', '500', '600', '700'],
  subsets: ['latin'],
  display: 'swap',
  variable: '--font-rajdhani',
});

const robotoMono = Roboto_Mono({
  weight: ['300', '400', '500', '600', '700'],
  subsets: ['latin'],
  display: 'swap',
  variable: '--font-roboto-mono',
});

export default function RootLayout({ children }) {
  return (
    <html lang="en" className={`${rajdhani.variable} ${robotoMono.variable}`}>
      <head>
        <title>Deakin Rover Mission Control</title>
        <meta name="description" content="Mission Control GUI for Deakin Rover Team" />
      </head>
      <body>
        <ThemeProvider theme={theme}>
          <CssBaseline />
          <ROSProvider>
            {children}
          </ROSProvider>
        </ThemeProvider>
      </body>
    </html>
  );
}
