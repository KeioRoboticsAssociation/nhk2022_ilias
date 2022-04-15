const { defineConfig } = require('@vue/cli-service')
module.exports = defineConfig({
  outputDir: './www',
  publicPath: './',

  transpileDependencies: [
    'quasar'
  ],

  pluginOptions: {
    quasar: {
      importStrategy: 'kebab',
      rtlSupport: false
    }
  }
})
