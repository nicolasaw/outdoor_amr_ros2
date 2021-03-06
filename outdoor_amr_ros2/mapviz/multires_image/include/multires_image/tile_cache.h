// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef MULTIRES_IMAGE_TILE_CACHE_H_
#define MULTIRES_IMAGE_TILE_CACHE_H_

// C++ standard libraries
#include <vector>
#include <stack>
#include <queue>
#include <map>

// QT libraries
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QGLWidget>

#include <tf2/transform_datatypes.h>

#include <multires_image/tile_set.h>
#include <multires_image/tile.h>

namespace multires_image
{
  class TileCache : public QObject
  {
  Q_OBJECT

  public:
    TileCache(TileSet* tileSet, QGLWidget* widget);
    ~TileCache() override;

    void Load(Tile* tile);
    void Precache(const tf2::Vector3& position);
    void Precache(double x, double y);

    void SetCurrentLayer(int layer) { m_currentLayer = layer; }

    void Exit();

  public Q_SLOTS:
    void LoadTextureSlot(Tile*);
    void DeleteTextureSlot(Tile*);

  Q_SIGNALS:
    void SignalLoadTexture(Tile*);
    void SignalDeleteTexture(Tile*);
    void SignalMemorySize(int64_t);

  private:
    TileSet*                  m_tileSet;
    QGLWidget*                m_widget;
    int32_t                   m_currentLayer;
    tf2::Vector3                 m_currentPosition;
    bool                      m_exit;
    int64_t                   m_memorySize;

    std::vector<std::queue<Tile*> > m_precacheRequests;
    std::stack<Tile*>               m_renderRequests;
    std::map<int64_t, Tile*>        m_textureLoaded;
    std::map<int64_t, Tile*>        m_renderRequestSet;
    std::map<int64_t, Tile*>        m_precacheRequestSet;

    void PrecacheLayer(int layer, const tf2::Vector3& position, int size);
    void LoadTexture(Tile* tile);
    void UnloadTexture(Tile* tile);

    class CacheThread : public QThread
    {
    public:
      explicit CacheThread(TileCache* parent) : p(parent) {}
      virtual void run();

    private:
      TileCache* p;
    };
    friend class CacheThread;

    class FreeThread : public QThread
    {
    public:
      explicit FreeThread(TileCache* parent) : p(parent) {}
      virtual void run();

    private:
      TileCache* p;
    };
    friend class FreeThread;

    CacheThread m_cacheThread;
    FreeThread  m_freeThread;

    QMutex      m_renderRequestsLock;
    QMutex      m_renderRequestSetLock;
    QMutex      m_precacheRequestsLock;
    QMutex      m_precacheRequestSetLock;
    QMutex      m_textureLoadedLock;
  };
}

#endif  // MULTIRES_IMAGE_TILE_CACHE_H_
