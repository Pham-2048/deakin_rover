'use client';

import { createTheme } from '@mui/material/styles';

// Iron Man inspired color palette
const colors = {
  background: {
    primary: '#0a0e27',
    secondary: '#121836',
    tertiary: '#1a2045',
  },
  accent: {
    cyan: '#00d9ff',
    cyanDark: '#0099cc',
    cyanLight: '#33e0ff',
  },
  error: {
    primary: '#ff0055',
    secondary: '#cc0044',
  },
  success: {
    primary: '#00ff88',
    secondary: '#00cc6a',
  },
  warning: {
    primary: '#ffaa00',
    secondary: '#cc8800',
  },
  text: {
    primary: '#e8f1ff',
    secondary: '#a8b8d8',
    disabled: '#6a7a9a',
  },
};

const theme = createTheme({
  palette: {
    mode: 'dark',
    primary: {
      main: colors.accent.cyan,
      dark: colors.accent.cyanDark,
      light: colors.accent.cyanLight,
    },
    secondary: {
      main: colors.background.secondary,
      dark: colors.background.primary,
      light: colors.background.tertiary,
    },
    error: {
      main: colors.error.primary,
      dark: colors.error.secondary,
    },
    success: {
      main: colors.success.primary,
      dark: colors.success.secondary,
    },
    warning: {
      main: colors.warning.primary,
      dark: colors.warning.secondary,
    },
    background: {
      default: colors.background.primary,
      paper: colors.background.secondary,
    },
    text: {
      primary: colors.text.primary,
      secondary: colors.text.secondary,
      disabled: colors.text.disabled,
    },
  },
  typography: {
    fontFamily: '"Rajdhani", "Roboto", "Helvetica", "Arial", sans-serif',
    h1: {
      fontFamily: '"Rajdhani", sans-serif',
      fontWeight: 700,
      fontSize: '3rem',
      letterSpacing: '0.05em',
      textTransform: 'uppercase',
    },
    h2: {
      fontFamily: '"Rajdhani", sans-serif',
      fontWeight: 600,
      fontSize: '2.5rem',
      letterSpacing: '0.05em',
      textTransform: 'uppercase',
    },
    h3: {
      fontFamily: '"Rajdhani", sans-serif',
      fontWeight: 600,
      fontSize: '2rem',
      letterSpacing: '0.05em',
      textTransform: 'uppercase',
    },
    h4: {
      fontFamily: '"Rajdhani", sans-serif',
      fontWeight: 600,
      fontSize: '1.5rem',
      letterSpacing: '0.05em',
      textTransform: 'uppercase',
    },
    h5: {
      fontFamily: '"Rajdhani", sans-serif',
      fontWeight: 600,
      fontSize: '1.25rem',
      letterSpacing: '0.05em',
      textTransform: 'uppercase',
    },
    h6: {
      fontFamily: '"Rajdhani", sans-serif',
      fontWeight: 600,
      fontSize: '1rem',
      letterSpacing: '0.05em',
      textTransform: 'uppercase',
    },
    body1: {
      fontFamily: '"Roboto Mono", monospace',
      fontSize: '0.95rem',
    },
    body2: {
      fontFamily: '"Roboto Mono", monospace',
      fontSize: '0.875rem',
    },
    button: {
      fontFamily: '"Rajdhani", sans-serif',
      fontWeight: 600,
      letterSpacing: '0.1em',
      textTransform: 'uppercase',
    },
  },
  components: {
    MuiCard: {
      styleOverrides: {
        root: {
          backgroundColor: colors.background.secondary,
          borderRadius: '8px',
          border: `1px solid ${colors.accent.cyan}33`,
          boxShadow: `0 0 20px ${colors.accent.cyan}22, inset 0 0 20px ${colors.accent.cyan}11`,
          transition: 'all 0.3s ease',
          '&:hover': {
            borderColor: `${colors.accent.cyan}66`,
            boxShadow: `0 0 30px ${colors.accent.cyan}44, inset 0 0 30px ${colors.accent.cyan}22`,
          },
        },
      },
    },
    MuiButton: {
      styleOverrides: {
        root: {
          borderRadius: '4px',
          padding: '10px 24px',
          border: `1px solid ${colors.accent.cyan}`,
          boxShadow: `0 0 10px ${colors.accent.cyan}44`,
          transition: 'all 0.2s ease',
          '&:hover': {
            backgroundColor: colors.accent.cyan + '22',
            boxShadow: `0 0 20px ${colors.accent.cyan}88`,
            transform: 'translateY(-2px)',
          },
          '&:active': {
            transform: 'translateY(0)',
          },
        },
        contained: {
          backgroundColor: colors.accent.cyan,
          color: colors.background.primary,
          '&:hover': {
            backgroundColor: colors.accent.cyanLight,
          },
        },
        outlined: {
          borderColor: colors.accent.cyan,
          color: colors.accent.cyan,
        },
      },
    },
    MuiTextField: {
      styleOverrides: {
        root: {
          '& .MuiOutlinedInput-root': {
            backgroundColor: colors.background.tertiary,
            '& fieldset': {
              borderColor: colors.accent.cyan + '44',
              borderWidth: '1px',
            },
            '&:hover fieldset': {
              borderColor: colors.accent.cyan + '88',
            },
            '&.Mui-focused fieldset': {
              borderColor: colors.accent.cyan,
              boxShadow: `0 0 10px ${colors.accent.cyan}44`,
            },
          },
          '& .MuiInputLabel-root': {
            color: colors.text.secondary,
            '&.Mui-focused': {
              color: colors.accent.cyan,
            },
          },
        },
      },
    },
    MuiChip: {
      styleOverrides: {
        root: {
          fontFamily: '"Rajdhani", sans-serif',
          fontWeight: 600,
          letterSpacing: '0.05em',
        },
        colorSuccess: {
          backgroundColor: colors.success.primary + '33',
          color: colors.success.primary,
          border: `1px solid ${colors.success.primary}`,
          boxShadow: `0 0 10px ${colors.success.primary}44`,
        },
        colorError: {
          backgroundColor: colors.error.primary + '33',
          color: colors.error.primary,
          border: `1px solid ${colors.error.primary}`,
          boxShadow: `0 0 10px ${colors.error.primary}44`,
        },
        colorWarning: {
          backgroundColor: colors.warning.primary + '33',
          color: colors.warning.primary,
          border: `1px solid ${colors.warning.primary}`,
          boxShadow: `0 0 10px ${colors.warning.primary}44`,
        },
      },
    },
    MuiPaper: {
      styleOverrides: {
        root: {
          backgroundColor: colors.background.secondary,
          backgroundImage: 'none',
        },
      },
    },
  },
});

export default theme;
